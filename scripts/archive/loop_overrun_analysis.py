"""
MRT3216 — Loop Overrun Deep Dive
=================================
Analyzes loop timing across multiple wpilog files to identify the root cause
of loop overruns. For each log, extracts:

1. Loop period (real cycle time) statistics
2. Correlation of slow loops with subsystem activity
3. Odometry thread timing (Phoenix 250Hz)
4. Vision processing load
5. CAN utilization signals
6. GC-indicative timing spikes
7. Timeline of overruns vs. robot state (auto/teleop/disabled)

Processes all recent logs and produces a comparative summary.
"""

import os
import sys
import time
import datetime
import statistics
from collections import defaultdict
from wpiutil.log import DataLogReader

# ── Configuration ───────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PATH = os.path.join(SCRIPT_DIR, "loop_overrun_analysis.txt")

# Collect logs from last 24h
NOW = time.time()
CUTOFF = NOW - 48 * 3600  # widen to 48h to catch the pre-match pit sessions

ROOTS = [
    os.path.expanduser("~/Desktop/Logs2"),
    "D:\\logs",
]

# Skip duplicates and tiny files
MIN_SIZE = 500_000  # 500 KB minimum

_out = None
def p(*a, **kw):
    print(*a, **kw)
    if _out:
        print(*a, **kw, file=_out)
        _out.flush()

def section(title):
    p(f"\n{'='*80}")
    p(f"  {title}")
    p(f"{'='*80}")

def subsection(title):
    p(f"\n  {'─'*70}")
    p(f"  {title}")
    p(f"  {'─'*70}")


def find_logs():
    """Find all .wpilog files modified recently, deduplicated by filename."""
    seen = set()
    logs = []
    for root in ROOTS:
        if not os.path.exists(root):
            continue
        for dirpath, _, filenames in os.walk(root):
            for f in filenames:
                if not f.endswith(".wpilog"):
                    continue
                full = os.path.join(dirpath, f)
                size = os.path.getsize(full)
                mtime = os.path.getmtime(full)
                if mtime < CUTOFF or size < MIN_SIZE:
                    continue
                if f in seen:
                    continue
                seen.add(f)
                logs.append((full, f, size, mtime))
    logs.sort(key=lambda x: x[3])
    return logs


def analyze_log(log_path):
    """Extract loop timing and subsystem data from a single wpilog."""
    entry_map = {}
    type_map = {}
    
    # Data we want to collect
    loop_times = []         # [(t, period_ms)]
    battery_volts = []      # [(t, volts)]
    can_util = []           # [(t, pct)]
    enabled_pts = []        # [(t, bool)]
    auto_pts = []           # [(t, bool)]
    
    # Per-subsystem timing (if AdvantageKit logs it)
    subsys_times = defaultdict(list)  # name -> [(t, ms)]
    
    # Odometry timestamps
    odom_timestamps = []    # [(t, [timestamps])]
    
    # Vision frame counts
    vision_has_target = []  # [(t, bool)]
    vision_tag_count = []   # [(t, count)]
    
    # Track all numeric keys for subsystem correlation
    numeric_data = defaultdict(list)
    
    for record in DataLogReader(log_path):
        if record.isStart():
            d = record.getStartData()
            entry_map[d.entry] = d.name
            type_map[d.entry] = d.type
            continue
        
        eid = record.getEntry()
        if eid not in entry_map:
            continue
        name = entry_map[eid]
        t = record.getTimestamp() / 1e6  # microseconds -> seconds
        typ = type_map[eid]
        
        try:
            if typ in ("double", "float"):
                val = record.getDouble()
                
                # Loop period
                if "LoggedRobot/FullCycleMS" in name or "Robot/FullCycleMS" in name:
                    loop_times.append((t, val))
                elif "RealCycleTime" in name or "LoopCycleTime" in name or "LoopTime" in name:
                    loop_times.append((t, val * 1000 if val < 1 else val))  # handle seconds vs ms
                
                # Battery
                elif "BatteryVoltage" in name or "SystemStats/BatteryVoltage" in name:
                    battery_volts.append((t, val))
                
                # CAN utilization
                elif "CANBus/Utilization" in name or "canUtil" in name:
                    can_util.append((t, val))
                
                # Per-subsystem timing logged by AdvantageKit
                elif "/PeriodicTime" in name or "/periodicMs" in name:
                    subsys_times[name].append((t, val))
                
                # Any timing-related key
                elif "MS" in name and "Cycle" in name:
                    numeric_data[name].append((t, val))
                    
            elif typ == "boolean":
                val = record.getBoolean()
                if name == "/DriverStation/Enabled":
                    enabled_pts.append((t, val))
                elif name == "/DriverStation/Autonomous":
                    auto_pts.append((t, val))
                elif "Vision/Summary/HasTarget" in name:
                    vision_has_target.append((t, val))
                    
            elif typ == "int64":
                val = record.getInteger()
                if "Vision/Summary/TagCount" in name:
                    vision_tag_count.append((t, val))
                    
            elif typ == "double[]":
                arr = list(record.getDoubleArray())
                if "OdometryTimestamps" in name:
                    odom_timestamps.append((t, arr))
                    
        except Exception:
            pass
    
    return {
        "loop_times": loop_times,
        "battery_volts": battery_volts,
        "can_util": can_util,
        "enabled_pts": enabled_pts,
        "auto_pts": auto_pts,
        "subsys_times": dict(subsys_times),
        "odom_timestamps": odom_timestamps,
        "vision_has_target": vision_has_target,
        "vision_tag_count": vision_tag_count,
        "numeric_data": dict(numeric_data),
    }


def compute_stats(values):
    """Compute summary statistics for a list of numeric values."""
    if not values:
        return None
    s = sorted(values)
    n = len(s)
    return {
        "n": n,
        "min": s[0],
        "max": s[-1],
        "mean": statistics.mean(s),
        "median": statistics.median(s),
        "stdev": statistics.stdev(s) if n > 1 else 0,
        "p95": s[int(0.95 * n)] if n > 20 else s[-1],
        "p99": s[int(0.99 * n)] if n > 100 else s[-1],
    }


def find_phases(enabled_pts, auto_pts):
    """Find auto/teleop start and end times."""
    auto_start = auto_end = teleop_start = teleop_end = None
    
    for t, v in enabled_pts:
        if v and auto_start is None:
            au_state = False
            for at, av in auto_pts:
                if at <= t:
                    au_state = av
            if au_state:
                auto_start = t
    
    if auto_start:
        for t, v in enabled_pts:
            if not v and t > auto_start and auto_end is None:
                auto_end = t
    
    for t, v in enabled_pts:
        if v and auto_end and t > auto_end and teleop_start is None:
            teleop_start = t
    
    for t, v in enabled_pts:
        if not v and teleop_start and t > teleop_start:
            teleop_end = t
    
    return auto_start, auto_end, teleop_start, teleop_end


def analyze_overrun_correlation(loop_times, battery_volts, can_util, vision_tag_count, odom_timestamps):
    """Correlate slow loops with other signals at the same time."""
    if not loop_times:
        return {}
    
    # Build time-indexed lookup for other signals
    def nearest_val(signal, target_t, max_delta=0.1):
        """Find nearest signal value within max_delta seconds."""
        best = None
        best_dt = float('inf')
        for t, v in signal:
            dt = abs(t - target_t)
            if dt < best_dt:
                best_dt = dt
                best = v
        return best if best_dt < max_delta else None
    
    # Classify loops
    fast = [v for _, v in loop_times if v <= 20]
    slow = [(t, v) for t, v in loop_times if v > 20]
    very_slow = [(t, v) for t, v in loop_times if v > 40]
    
    # For slow loops, check battery voltage at that moment
    slow_battery = []
    for t, v in slow[:200]:  # sample up to 200
        bv = nearest_val(battery_volts, t, 0.5)
        if bv is not None:
            slow_battery.append(bv)
    
    fast_battery = []
    for t, v in loop_times[:200]:
        if v <= 20:
            bv = nearest_val(battery_volts, t, 0.5)
            if bv is not None:
                fast_battery.append(bv)
    
    return {
        "fast_count": len(fast),
        "slow_count": len(slow),
        "very_slow_count": len(very_slow),
        "slow_battery_mean": statistics.mean(slow_battery) if slow_battery else None,
        "fast_battery_mean": statistics.mean(fast_battery) if fast_battery else None,
        "very_slow_times": very_slow[:20],  # worst offenders
    }


def main():
    global _out
    _out = open(OUTPUT_PATH, "w", encoding="utf-8")
    
    p("MRT3216 — Loop Overrun Deep Dive Analysis")
    p(f"Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    logs = find_logs()
    p(f"\nFound {len(logs)} log files to analyze")
    
    # ── Per-log analysis ──
    all_results = []
    
    for log_path, fname, size, mtime in logs:
        dt = datetime.datetime.fromtimestamp(mtime)
        is_match = "_idbo" in fname or "_azfg" in fname
        label = "MATCH" if is_match else "PIT/TEST"
        
        p(f"\n  Parsing: {fname} ({size/1024/1024:.1f} MB, {label}, {dt.strftime('%m-%d %H:%M')})")
        
        try:
            data = analyze_log(log_path)
        except Exception as e:
            p(f"    ⚠ Error parsing: {e}")
            continue
        
        loop_vals = [v for _, v in data["loop_times"]]
        stats = compute_stats(loop_vals)
        
        if stats is None or stats["n"] < 50:
            p(f"    Skipped — only {len(loop_vals)} loop samples")
            continue
        
        phases = find_phases(data["enabled_pts"], data["auto_pts"])
        
        overruns_20 = sum(1 for v in loop_vals if v > 20)
        overruns_30 = sum(1 for v in loop_vals if v > 30)
        overruns_50 = sum(1 for v in loop_vals if v > 50)
        
        all_results.append({
            "fname": fname,
            "label": label,
            "dt": dt,
            "stats": stats,
            "overruns_20": overruns_20,
            "overruns_30": overruns_30,
            "overruns_50": overruns_50,
            "data": data,
            "phases": phases,
        })
        
        p(f"    Loops: {stats['n']:,}  mean={stats['mean']:.1f}ms  p95={stats['p95']:.1f}ms  "
          f"p99={stats['p99']:.1f}ms  max={stats['max']:.1f}ms")
        p(f"    Overruns: >20ms={overruns_20} ({100*overruns_20/stats['n']:.1f}%)  "
          f">30ms={overruns_30}  >50ms={overruns_50}")
    
    if not all_results:
        p("\n  No valid logs found!")
        _out.close()
        return
    
    # ── Comparative summary ──
    section("COMPARATIVE LOOP TIMING SUMMARY")
    p(f"\n  {'File':<45s}  {'Type':>5s}  {'Loops':>7s}  {'Mean':>6s}  {'p95':>6s}  "
      f"{'p99':>6s}  {'Max':>7s}  {'>20ms':>7s}  {'>30ms':>6s}  {'>50ms':>6s}")
    p(f"  {'-'*45}  {'---':>5s}  {'---':>7s}  {'---':>6s}  {'---':>6s}  "
      f"{'---':>6s}  {'---':>7s}  {'---':>7s}  {'---':>6s}  {'---':>6s}")
    
    for r in all_results:
        s = r["stats"]
        pct = f"{100*r['overruns_20']/s['n']:.0f}%"
        p(f"  {r['fname'][:45]:<45s}  {r['label']:>5s}  {s['n']:>7,}  {s['mean']:>5.1f}ms  "
          f"{s['p95']:>5.1f}ms  {s['p99']:>5.1f}ms  {s['max']:>6.1f}ms  "
          f"{pct:>7s}  {r['overruns_30']:>6d}  {r['overruns_50']:>6d}")
    
    # ── Detailed analysis of worst logs ──
    # Sort by overrun rate
    by_overrun = sorted(all_results, key=lambda r: r["overruns_20"] / r["stats"]["n"], reverse=True)
    
    section("DETAILED ANALYSIS — TOP 5 WORST LOGS BY OVERRUN RATE")
    
    for rank, r in enumerate(by_overrun[:5], 1):
        subsection(f"#{rank}: {r['fname']} ({r['label']})")
        s = r["stats"]
        data = r["data"]
        
        p(f"    Loop cycles: {s['n']:,}")
        p(f"    mean={s['mean']:.2f}ms  median={s['median']:.2f}ms  σ={s['stdev']:.2f}ms")
        p(f"    p95={s['p95']:.2f}ms  p99={s['p99']:.2f}ms  max={s['max']:.2f}ms")
        p(f"    >20ms: {r['overruns_20']} ({100*r['overruns_20']/s['n']:.1f}%)")
        p(f"    >30ms: {r['overruns_30']} ({100*r['overruns_30']/s['n']:.1f}%)")
        p(f"    >50ms: {r['overruns_50']} ({100*r['overruns_50']/s['n']:.1f}%)")
        
        # Histogram
        loop_vals = [v for _, v in data["loop_times"]]
        buckets = [0, 10, 15, 20, 25, 30, 40, 50, 75, 100, 200, 500]
        p(f"\n    Distribution:")
        for i in range(len(buckets) - 1):
            count = sum(1 for v in loop_vals if buckets[i] <= v < buckets[i+1])
            bar = "█" * min(count * 60 // max(s["n"], 1), 60)
            if count > 0:
                p(f"      {buckets[i]:>4d}-{buckets[i+1]:>4d}ms: {count:>6d} ({100*count/s['n']:>5.1f}%) {bar}")
        over = sum(1 for v in loop_vals if v >= buckets[-1])
        if over > 0:
            p(f"      {buckets[-1]:>4d}+   ms: {over:>6d} ({100*over/s['n']:>5.1f}%)")
        
        # Phase breakdown
        AS, AE, TS, TE = r["phases"]
        if TS and TE:
            tele_loops = [(t, v) for t, v in data["loop_times"] if TS <= t <= TE]
            tele_vals = [v for _, v in tele_loops]
            if tele_vals:
                tele_stats = compute_stats(tele_vals)
                tele_over = sum(1 for v in tele_vals if v > 20)
                p(f"\n    Teleop only: {tele_stats['n']:,} loops, "
                  f"mean={tele_stats['mean']:.1f}ms, p95={tele_stats['p95']:.1f}ms, "
                  f">20ms={tele_over} ({100*tele_over/tele_stats['n']:.1f}%)")
        
        if AS and AE:
            auto_loops = [(t, v) for t, v in data["loop_times"] if AS <= t <= AE]
            auto_vals = [v for _, v in auto_loops]
            if auto_vals:
                auto_stats = compute_stats(auto_vals)
                auto_over = sum(1 for v in auto_vals if v > 20)
                p(f"    Auto only:   {auto_stats['n']:,} loops, "
                  f"mean={auto_stats['mean']:.1f}ms, p95={auto_stats['p95']:.1f}ms, "
                  f">20ms={auto_over} ({100*auto_over/auto_stats['n']:.1f}%)")
        
        disabled_loops = []
        if data["loop_times"]:
            # Before first enable or after last disable
            first_enable = None
            for t, v in data["enabled_pts"]:
                if v:
                    first_enable = t
                    break
            if first_enable:
                disabled_loops = [(t, v) for t, v in data["loop_times"] if t < first_enable]
            disabled_vals = [v for _, v in disabled_loops]
            if disabled_vals and len(disabled_vals) > 20:
                dis_stats = compute_stats(disabled_vals)
                dis_over = sum(1 for v in disabled_vals if v > 20)
                p(f"    Disabled:    {dis_stats['n']:,} loops, "
                  f"mean={dis_stats['mean']:.1f}ms, p95={dis_stats['p95']:.1f}ms, "
                  f">20ms={dis_over} ({100*dis_over/dis_stats['n']:.1f}%)")
        
        # Battery correlation
        bv_vals = [v for _, v in data["battery_volts"]]
        if bv_vals:
            p(f"\n    Battery: min={min(bv_vals):.2f}V  mean={statistics.mean(bv_vals):.2f}V  max={max(bv_vals):.2f}V")
        
        # CAN utilization
        can_vals = [v for _, v in data["can_util"]]
        if can_vals:
            p(f"    CAN util: min={min(can_vals):.1f}%  mean={statistics.mean(can_vals):.1f}%  max={max(can_vals):.1f}%")
        
        # Vision tag count during teleop
        if data["vision_tag_count"] and TS and TE:
            tc_tele = [v for t, v in data["vision_tag_count"] if TS <= t <= TE]
            if tc_tele:
                p(f"    Vision tags (teleop): mean={statistics.mean(tc_tele):.1f}  max={max(tc_tele)}")
        
        # Odometry thread analysis
        if data["odom_timestamps"]:
            odom_counts = [len(arr) for _, arr in data["odom_timestamps"]]
            if odom_counts:
                p(f"    Odometry samples/cycle: mean={statistics.mean(odom_counts):.1f}  "
                  f"max={max(odom_counts)}  min={min(odom_counts)}")
        
        # Overrun clustering: find bursts of consecutive overruns
        loop_ts = data["loop_times"]
        overrun_bursts = []
        burst_start = None
        burst_count = 0
        for t, v in loop_ts:
            if v > 20:
                if burst_start is None:
                    burst_start = t
                burst_count += 1
            else:
                if burst_count >= 3:
                    overrun_bursts.append((burst_start, t, burst_count))
                burst_start = None
                burst_count = 0
        if burst_count >= 3:
            overrun_bursts.append((burst_start, loop_ts[-1][0], burst_count))
        
        if overrun_bursts:
            p(f"\n    Overrun bursts (≥3 consecutive >20ms):")
            for bs, be, bc in overrun_bursts[:10]:
                p(f"      t={bs:.1f}s → {be:.1f}s ({be-bs:.1f}s, {bc} cycles)")
            if len(overrun_bursts) > 10:
                p(f"      ... {len(overrun_bursts)} total bursts")
        
        # Worst individual loops
        worst = sorted(data["loop_times"], key=lambda x: x[1], reverse=True)[:10]
        p(f"\n    10 worst individual loops:")
        for t, v in worst:
            phase = "?"
            if AS and AE and AS <= t <= AE:
                phase = "AUTO"
            elif TS and TE and TS <= t <= TE:
                phase = "TELE"
            else:
                phase = "DIS"
            extra = ""
            # Check battery at that moment
            for bt, bv in data["battery_volts"]:
                if abs(bt - t) < 0.1:
                    extra = f"  batt={bv:.1f}V"
                    break
            p(f"      t={t:>8.2f}s  {v:>7.2f}ms  [{phase}]{extra}")
        
        # Subsystem timing if available
        if data["subsys_times"]:
            p(f"\n    Per-subsystem periodic times:")
            for key, pts in sorted(data["subsys_times"].items()):
                vals = [v for _, v in pts]
                st = compute_stats(vals)
                if st and st["max"] > 1:
                    p(f"      {key}: mean={st['mean']:.2f}ms  p95={st['p95']:.2f}ms  max={st['max']:.2f}ms")
    
    # ── Pattern analysis ──
    section("PATTERN ANALYSIS")
    
    pit_results = [r for r in all_results if r["label"] == "PIT/TEST"]
    match_results = [r for r in all_results if r["label"] == "MATCH"]
    
    if pit_results:
        pit_means = [r["stats"]["mean"] for r in pit_results]
        pit_p95s = [r["stats"]["p95"] for r in pit_results]
        pit_rates = [100 * r["overruns_20"] / r["stats"]["n"] for r in pit_results]
        p(f"\n  PIT/TEST logs ({len(pit_results)}):")
        p(f"    Mean loop time: {min(pit_means):.1f} – {max(pit_means):.1f}ms (avg {statistics.mean(pit_means):.1f}ms)")
        p(f"    p95 loop time:  {min(pit_p95s):.1f} – {max(pit_p95s):.1f}ms (avg {statistics.mean(pit_p95s):.1f}ms)")
        p(f"    Overrun rate:   {min(pit_rates):.1f}% – {max(pit_rates):.1f}% (avg {statistics.mean(pit_rates):.1f}%)")
    
    if match_results:
        match_means = [r["stats"]["mean"] for r in match_results]
        match_p95s = [r["stats"]["p95"] for r in match_results]
        match_rates = [100 * r["overruns_20"] / r["stats"]["n"] for r in match_results]
        p(f"\n  MATCH logs ({len(match_results)}):")
        p(f"    Mean loop time: {min(match_means):.1f} – {max(match_means):.1f}ms (avg {statistics.mean(match_means):.1f}ms)")
        p(f"    p95 loop time:  {min(match_p95s):.1f} – {max(match_p95s):.1f}ms (avg {statistics.mean(match_p95s):.1f}ms)")
        p(f"    Overrun rate:   {min(match_rates):.1f}% – {max(match_rates):.1f}% (avg {statistics.mean(match_rates):.1f}%)")
    
    if pit_results and match_results:
        pit_avg = statistics.mean([r["stats"]["mean"] for r in pit_results])
        match_avg = statistics.mean([r["stats"]["mean"] for r in match_results])
        diff = match_avg - pit_avg
        p(f"\n  ΔMatch vs Pit: {diff:+.1f}ms mean loop time")
        if diff > 3:
            p(f"    → Match loops are significantly slower — suggests FMS/field-specific load")
        elif diff > 1:
            p(f"    → Match loops are somewhat slower — possible network/FMS overhead")
        else:
            p(f"    → Match and pit loops are similar — overrun cause is intrinsic to code")
    
    # ── Timing key discovery ──
    section("AVAILABLE TIMING KEYS")
    p("  (Keys containing 'MS', 'Time', 'Cycle', 'Period' found across all logs)")
    all_timing_keys = set()
    for r in all_results:
        for key in r["data"]["numeric_data"]:
            all_timing_keys.add(key)
        for key in r["data"]["subsys_times"]:
            all_timing_keys.add(key)
    # Also check loop_times key names
    # Re-scan one log to find the actual key name
    if all_results:
        sample_path = find_logs()[0][0]
        for record in DataLogReader(sample_path):
            if record.isStart():
                d = record.getStartData()
                name = d.name
                if any(x in name for x in ["MS", "Time", "Cycle", "Period", "Loop", "periodic", "Odometry"]):
                    all_timing_keys.add(f"[{d.type}] {name}")
            # Only scan starts
    
    for key in sorted(all_timing_keys):
        p(f"    {key}")
    
    p(f"\n  Output written to: {OUTPUT_PATH}")
    _out.close()


if __name__ == "__main__":
    main()
