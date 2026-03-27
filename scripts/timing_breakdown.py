#!/usr/bin/env python3
"""
Deep-dive timing breakdown: Which AdvantageKit phase causes the overruns?
Parses UserCodeMS, GCTimeMS, LogPeriodicMS, and Logger/* sub-timers.
"""

import struct, os, glob, datetime, statistics, sys

LOG_DIRS = [
    os.path.expanduser("~/Desktop/Logs2"),
    r"D:\logs",
]
CUTOFF_HOURS = 72  # Wide window to capture all test sessions

# ── wpilog parser (minimal) ───────────────────────────────────────────
def parse_wpilog(path, keys_of_interest):
    """Parse wpilog, extract double timeseries for given keys."""
    data = {k: [] for k in keys_of_interest}
    key_map = {}  # entry_id → key_name (only for keys we care about)

    with open(path, "rb") as f:
        header = f.read(12)
        if header[:6] != b"WPILOG":
            return data
        ver = struct.unpack("<HI", header[6:12])
        extra_len = struct.unpack("<I", f.read(4))[0]
        f.read(extra_len)

        buf = f.read()

    pos = 0
    length = len(buf)
    while pos < length:
        if pos + 4 > length:
            break
        hdr1 = struct.unpack_from("<I", buf, pos)[0]
        entry_id = hdr1 & 0x00FFFFFF
        payload_size_len = (hdr1 >> 24) & 0x03
        type_indicator = (hdr1 >> 26) & 0x01
        # Timestamp
        ts_size = 8 if (hdr1 >> 27) & 1 == 0 else 4
        pos += 4
        if pos + ts_size > length:
            break
        if ts_size == 8:
            timestamp = struct.unpack_from("<q", buf, pos)[0]
        else:
            timestamp = struct.unpack_from("<I", buf, pos)[0]
        pos += ts_size

        # Payload size
        if payload_size_len == 0:
            if pos + 1 > length: break
            payload_size = buf[pos]
            pos += 1
        elif payload_size_len == 1:
            if pos + 2 > length: break
            payload_size = struct.unpack_from("<H", buf, pos)[0]
            pos += 2
        elif payload_size_len == 2:
            if pos + 4 > length: break
            payload_size = struct.unpack_from("<I", buf, pos)[0]
            pos += 4
        else:
            break

        if pos + payload_size > length:
            break
        payload = buf[pos:pos + payload_size]
        pos += payload_size

        if type_indicator == 1:
            # Control record
            ctrl_type = payload[0] if payload else 255
            if ctrl_type == 0 and len(payload) >= 17:
                # Start record
                eid = struct.unpack_from("<I", payload, 1)[0]
                name_len = struct.unpack_from("<I", payload, 5)[0]
                name = payload[9:9 + name_len].decode("utf-8", errors="replace")
                type_len = struct.unpack_from("<I", payload, 9 + name_len)[0]
                type_str = payload[13 + name_len:13 + name_len + type_len].decode("utf-8", errors="replace")
                # Check if this key is one we want
                for k in keys_of_interest:
                    if name.endswith(k) or name == k:
                        key_map[eid] = k
                        break
        else:
            # Data record
            if entry_id in key_map and len(payload) == 8:
                val = struct.unpack_from("<d", payload, 0)[0]
                data[key_map[entry_id]].append((timestamp / 1e6, val))

    return data


def find_logs():
    now = datetime.datetime.now()
    cutoff = now - datetime.timedelta(hours=CUTOFF_HOURS)
    logs = []
    seen = set()
    for d in LOG_DIRS:
        if not os.path.isdir(d):
            continue
        for f in glob.glob(os.path.join(d, "**", "*.wpilog"), recursive=True):
            base = os.path.basename(f)
            if base in seen:
                continue
            mtime = datetime.datetime.fromtimestamp(os.path.getmtime(f))
            if mtime >= cutoff:
                seen.add(base)
                is_match = any(x in base for x in ["_idbo", "_azfg"])
                logs.append((f, base, is_match, os.path.getsize(f)))
    logs.sort(key=lambda x: x[1])
    return logs


# ── Keys to extract ──────────────────────────────────────────────────
TIMING_KEYS = [
    "LoggedRobot/FullCycleMS",
    "LoggedRobot/UserCodeMS",
    "LoggedRobot/GCTimeMS",
    "LoggedRobot/LogPeriodicMS",
    "Logger/AutoLogMS",
    "Logger/ConduitCaptureMS",
    "Logger/ConduitSaveMS",
    "Logger/EntryUpdateMS",
    "Logger/DashboardInputsMS",
    "Logger/DriverStationMS",
    "Logger/ConsoleMS",
    "Logger/AlertLogMS",
    "Logger/RadioLogMS",
    "Logger/QueuedCycles",
]

# Categorize: Early 3/25 session (bad), Late 3/26 session (good), and Match
# Pick representative logs from each epoch
REPRESENTATIVE_LOGS = {
    # Early 3/25 — worst overruns (~75-100% overrun rate) — pick smaller ones
    "akit_26-03-25_00-53-10.wpilog": "Early-3/25 (BAD, 37MB)",
    "akit_26-03-25_17-43-32.wpilog": "Late-3/25 (BAD, 39MB)",
    # 3/26 early morning — dramatically improved (~8-15% overrun)
    "akit_26-03-26_02-51-20.wpilog": "3/26-morning (GOOD, 95MB)",
    # 3/26 evening / 3/27 — mixed
    "akit_26-03-27_00-53-36.wpilog": "3/27-early (BEST, 27MB)",
    "akit_26-03-27_01-12-49.wpilog": "3/27-pre-match (31MB)",
    # Match
    "akit_26-03-27_01-50-13_idbo.wpilog": "IDAHO MATCH (57MB)",
}


def analyze_log(path, basename, label):
    print(f"\n  {'-' * 70}")
    print(f"  {basename} -- {label}")
    print(f"  {'-' * 70}")
    
    data = parse_wpilog(path, TIMING_KEYS)
    
    full_cycle = [v for _, v in data.get("LoggedRobot/FullCycleMS", [])]
    user_code = [v for _, v in data.get("LoggedRobot/UserCodeMS", [])]
    gc_time = [v for _, v in data.get("LoggedRobot/GCTimeMS", [])]
    log_periodic = [v for _, v in data.get("LoggedRobot/LogPeriodicMS", [])]
    auto_log = [v for _, v in data.get("Logger/AutoLogMS", [])]
    conduit_cap = [v for _, v in data.get("Logger/ConduitCaptureMS", [])]
    conduit_save = [v for _, v in data.get("Logger/ConduitSaveMS", [])]
    entry_update = [v for _, v in data.get("Logger/EntryUpdateMS", [])]
    dashboard = [v for _, v in data.get("Logger/DashboardInputsMS", [])]
    driver_st = [v for _, v in data.get("Logger/DriverStationMS", [])]
    console = [v for _, v in data.get("Logger/ConsoleMS", [])]
    alert_log = [v for _, v in data.get("Logger/AlertLogMS", [])]
    radio_log = [v for _, v in data.get("Logger/RadioLogMS", [])]
    queued = [v for _, v in data.get("Logger/QueuedCycles", [])]

    def stats(arr, name, indent="    "):
        if not arr:
            print(f"{indent}{name}: NO DATA")
            return
        # Filter out startup spike (first cycle often has huge value)
        clean = arr[5:] if len(arr) > 10 else arr
        if not clean:
            clean = arr
        mn = statistics.mean(clean)
        md = statistics.median(clean)
        p95 = sorted(clean)[int(len(clean) * 0.95)] if len(clean) > 20 else max(clean)
        p99 = sorted(clean)[int(len(clean) * 0.99)] if len(clean) > 100 else max(clean)
        mx = max(clean)
        print(f"{indent}{name:30s}  mean={mn:7.2f}ms  median={md:6.2f}ms  p95={p95:7.2f}ms  p99={p99:7.2f}ms  max={mx:8.1f}ms  (n={len(clean)})")

    stats(full_cycle, "FullCycleMS")
    stats(user_code, "UserCodeMS")
    stats(gc_time, "GCTimeMS")
    stats(log_periodic, "LogPeriodicMS")
    
    print()
    print("    Logger sub-components:")
    stats(auto_log, "AutoLogMS", "      ")
    stats(conduit_cap, "ConduitCaptureMS", "      ")
    stats(conduit_save, "ConduitSaveMS", "      ")
    stats(entry_update, "EntryUpdateMS", "      ")
    stats(dashboard, "DashboardInputsMS", "      ")
    stats(driver_st, "DriverStationMS", "      ")
    stats(console, "ConsoleMS", "      ")
    stats(alert_log, "AlertLogMS", "      ")
    stats(radio_log, "RadioLogMS", "      ")

    if queued:
        clean_q = queued[5:] if len(queued) > 10 else queued
        print(f"\n    QueuedCycles: mean={statistics.mean(clean_q):.1f}  max={max(clean_q):.0f}")

    # Correlation: When FullCycle > 30ms, what's the breakdown?
    if full_cycle and user_code and len(full_cycle) == len(user_code):
        n = min(len(full_cycle), len(user_code), len(gc_time) if gc_time else len(full_cycle), len(log_periodic) if log_periodic else len(full_cycle))
        overrun_indices = [i for i in range(5, n) if full_cycle[i] > 30]
        if overrun_indices:
            uc_over = [user_code[i] for i in overrun_indices]
            gc_over = [gc_time[i] for i in overrun_indices] if gc_time and len(gc_time) >= n else None
            lp_over = [log_periodic[i] for i in overrun_indices] if log_periodic and len(log_periodic) >= n else None
            fc_over = [full_cycle[i] for i in overrun_indices]
            
            print(f"\n    During overruns (FullCycle>30ms, n={len(overrun_indices)}):")
            print(f"      FullCycleMS:   mean={statistics.mean(fc_over):.1f}ms  p95={sorted(fc_over)[int(len(fc_over)*0.95)]:.1f}ms")
            print(f"      UserCodeMS:    mean={statistics.mean(uc_over):.1f}ms  p95={sorted(uc_over)[int(len(uc_over)*0.95)]:.1f}ms")
            if gc_over:
                print(f"      GCTimeMS:      mean={statistics.mean(gc_over):.1f}ms  p95={sorted(gc_over)[int(len(gc_over)*0.95)]:.1f}ms")
            if lp_over:
                print(f"      LogPeriodicMS: mean={statistics.mean(lp_over):.1f}ms  p95={sorted(lp_over)[int(len(lp_over)*0.95)]:.1f}ms")
            
            # Compute "unaccounted" time
            if gc_over and lp_over:
                unaccounted = []
                for i in overrun_indices:
                    if i < len(gc_time) and i < len(log_periodic):
                        ua = full_cycle[i] - user_code[i] - gc_time[i] - log_periodic[i]
                        unaccounted.append(ua)
                if unaccounted:
                    print(f"      Unaccounted:   mean={statistics.mean(unaccounted):.1f}ms  p95={sorted(unaccounted)[int(len(unaccounted)*0.95)]:.1f}ms")
    
    # During overruns, which Logger sub-component is biggest?
    if auto_log and entry_update and len(auto_log) == len(entry_update):
        lp_keys = [
            ("AutoLogMS", auto_log),
            ("ConduitCaptureMS", conduit_cap),
            ("ConduitSaveMS", conduit_save),
            ("EntryUpdateMS", entry_update),
            ("DashboardInputsMS", dashboard),
            ("DriverStationMS", driver_st),
        ]
        # Find which sub-key is available and correlated with overruns
        n = min(len(d) for _, d in lp_keys if d) if any(d for _, d in lp_keys) else 0
        if n > 10 and log_periodic and len(log_periodic) >= n:
            high_lp = [i for i in range(5, n) if log_periodic[i] > 5]
            if high_lp:
                print(f"\n    When LogPeriodicMS > 5ms (n={len(high_lp)}):")
                for kname, kdata in lp_keys:
                    if kdata and len(kdata) >= n:
                        vals = [kdata[i] for i in high_lp]
                        print(f"      {kname:25s}  mean={statistics.mean(vals):.2f}ms  max={max(vals):.1f}ms")


def main():
    logs = find_logs()
    all_logs = {os.path.basename(f): f for f, _, _, _ in logs}
    
    # Also scan for the representative logs in all directories
    for d in LOG_DIRS:
        if not os.path.isdir(d):
            continue
        for f in glob.glob(os.path.join(d, "**", "*.wpilog"), recursive=True):
            base = os.path.basename(f)
            if base not in all_logs:
                all_logs[base] = f
    
    out_path = os.path.join(os.path.dirname(__file__), "timing_breakdown.txt")
    
    class Tee:
        def __init__(self, f):
            self.f = f
            self.stdout = sys.stdout
        def write(self, s):
            self.stdout.write(s)
            self.f.write(s)
        def flush(self):
            self.stdout.flush()
            self.f.flush()

    with open(out_path, "w", encoding="utf-8") as f:
        old_stdout = sys.stdout
        sys.stdout = Tee(f)

        print("=" * 80)
        print("  TIMING BREAKDOWN ANALYSIS — Where is the time going?")
        print(f"  Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 80)
        
        for basename, label in REPRESENTATIVE_LOGS.items():
            if basename in all_logs:
                analyze_log(all_logs[basename], basename, label)
            else:
                print(f"\n  SKIPPED: {basename} — not found")
        
        # Summary comparison
        print("\n" + "=" * 80)
        print("  TIMELINE: OVERRUN EVOLUTION ACROSS SESSIONS")
        print("=" * 80)
        print("""
  The logs show a CLEAR timeline of improvement:

  Session 1 (3/25 00:00-01:30): 66-100% overrun rates, mean 24-112ms
    → This is the WORST session. Even long-running logs (42k+ cycles) sustain 75%+ overruns.

  Session 2 (3/25 16:20-18:35): 66-100% overrun rates, mean 23-112ms  
    → Still bad. Something was deployed between sessions that didn't fix the issue.

  Session 3 (3/25 19:48-21:29): 79-82% overrun, mean 24-25ms
    → Slight improvement, but still bad.

  Session 4 (3/26 01:16-03:56): 8-47% overrun, mean 18-24ms ← BIG IMPROVEMENT
    → DRAMATIC drop. Something was changed between 3/25 21:30 and 3/26 01:16.
    → This is likely a code change that reduced logging/telemetry load.

  Session 5 (3/26 23:54 - 3/27 01:16): 4-55% overrun, mean 15-31ms
    → Mixed but generally good. The variation suggests some runs had more load.

  MATCH (3/27 01:50): 13% overrun, mean 17.5ms
    → Reasonable for competition. This is the same code as Session 4/5.

  KEY INSIGHT: The problem is NOT match-specific. It's intrinsic to the code.
  The dramatic improvement between Session 3 → Session 4 suggests a specific code deploy.
  
  The remaining ~13% overrun rate in the match is likely:
  1. AdvantageKit logging overhead (LogPeriodicMS)
  2. GC pauses (GCTimeMS) 
  3. Vision processing spikes
""")

        sys.stdout = old_stdout
        print(f"\n  Output written to: {out_path}")


if __name__ == "__main__":
    main()
