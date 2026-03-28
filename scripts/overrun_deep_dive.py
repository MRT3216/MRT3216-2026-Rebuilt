"""
MRT3216 — Deep Loop Overrun Investigation
==========================================
Analyzes the Boise competition wpilog files to identify root causes
of the ~40% loop overrun rate observed across matches.

Analysis includes:
  1. FullCycleMS breakdown: UserCode vs LogPeriodic vs Logger components
  2. Logger component breakdown (ConduitCapture, ConduitSave, AutoLog, etc.)
  3. Per-cycle correlation: which Logger components spike when overruns happen?
  4. CAN bus utilization vs cycle time correlation
  5. Vision tag count vs cycle time correlation
  6. Battery voltage vs cycle time correlation
  7. Active commands during worst overrun windows
  8. Temporal clustering: when in the match do overruns concentrate?
  9. Disabled vs Auto vs Teleop comparison
 10. Cross-match trends and actionable recommendations

Usage:
    .venv/Scripts/python.exe scripts/overrun_deep_dive.py "C:/Users/danla/Desktop/Logs Idaho"
"""

import sys, os, glob, math, statistics
from collections import defaultdict
from wpiutil.log import DataLogReader

if len(sys.argv) < 2:
    print("Usage: python overrun_deep_dive.py <log-directory>")
    sys.exit(1)

LOG_DIR = sys.argv[1]
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PATH = os.path.join(SCRIPT_DIR, "overrun_deep_dive.txt")
_out = open(OUTPUT_PATH, "w", encoding="utf-8")

LOOP_PERIOD_MS = 20.0
OVERRUN_THRESHOLD_MS = 20.0  # cycles > this count as overrun

def p(*args, **kwargs):
    print(*args, **kwargs)
    print(*args, **kwargs, file=_out)
    _out.flush()


# ══════════════════════════════════════════════════════════════════════════════
# Parsing
# ══════════════════════════════════════════════════════════════════════════════

def parse_wpilog(path):
    """Parse wpilog into typed dicts. Returns (numeric, booleans, strings, string_arrays, doubles_arr)."""
    entry_map = {}
    type_map = {}
    numeric = defaultdict(list)
    booleans = defaultdict(list)
    strings = defaultdict(list)
    string_arrays = defaultdict(list)
    doubles_arr = defaultdict(list)

    for record in DataLogReader(path):
        if record.isStart():
            d = record.getStartData()
            entry_map[d.entry] = d.name
            type_map[d.entry] = d.type
            continue
        eid = record.getEntry()
        if eid not in entry_map:
            continue
        name = entry_map[eid]
        t = record.getTimestamp() / 1e6
        typ = type_map[eid]
        try:
            if typ == "double":
                numeric[name].append((t, record.getDouble()))
            elif typ == "float":
                numeric[name].append((t, record.getFloat()))
            elif typ == "int64":
                numeric[name].append((t, float(record.getInteger())))
            elif typ == "boolean":
                booleans[name].append((t, record.getBoolean()))
            elif typ == "string":
                strings[name].append((t, record.getString()))
            elif typ == "string[]":
                string_arrays[name].append((t, list(record.getStringArray())))
            elif typ == "double[]":
                doubles_arr[name].append((t, list(record.getDoubleArray())))
        except:
            pass

    return numeric, booleans, strings, string_arrays, doubles_arr


def extract_label(path):
    """Extract match label from filename like akit_26-03-27_17-41-24_idbo_q6.wpilog."""
    base = os.path.splitext(os.path.basename(path))[0]
    parts = base.split("_")
    event = ""
    match_num = ""
    for part in parts:
        if part.lower().startswith("q") and part[1:].isdigit():
            match_num = part.upper()
        elif part.isalpha() and len(part) >= 3 and part.lower() not in ("akit",):
            event = part.upper()
    return f"{event} {match_num}".strip() or base


def find_phases(en_pts, au_pts):
    """Find auto/teleop start/end times."""
    auto_start = auto_end = teleop_start = teleop_end = None
    for t, v in en_pts:
        if v and auto_start is None:
            au_state = False
            for at, av in au_pts:
                if at <= t: au_state = av
            if au_state:
                auto_start = t
    if auto_start:
        for t, v in en_pts:
            if not v and t > auto_start and auto_end is None:
                auto_end = t
    for t, v in en_pts:
        if v and auto_end and t > auto_end and teleop_start is None:
            teleop_start = t
    for t, v in en_pts:
        if not v and teleop_start and t > teleop_start and teleop_end is None:
            teleop_end = t
    return auto_start, auto_end, teleop_start, teleop_end


def vals_in_range(pts, t0, t1):
    """Return list of values within [t0, t1]."""
    return [v for t, v in pts if t0 <= t <= t1]


def nearest_val(pts, target_t):
    """Return value closest to target_t."""
    if not pts:
        return None
    best = min(pts, key=lambda x: abs(x[0] - target_t))
    return best[1]


def percentile(data, pct):
    """Compute percentile (0-100)."""
    if not data:
        return 0.0
    s = sorted(data)
    k = (pct / 100) * (len(s) - 1)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return s[int(k)]
    return s[f] * (c - k) + s[c] * (k - f)


def build_time_index(pts):
    """Build sorted (t, v) list for binary search."""
    return sorted(pts, key=lambda x: x[0])


# ══════════════════════════════════════════════════════════════════════════════
# Per-match deep analysis
# ══════════════════════════════════════════════════════════════════════════════

# Timing keys we want to analyze
TIMING_KEYS = {
    "full":        "/RealOutputs/LoggedRobot/FullCycleMS",
    "user":        "/RealOutputs/LoggedRobot/UserCodeMS",
    "log_periodic":"/RealOutputs/LoggedRobot/LogPeriodicMS",
    "gc_time":     "/RealOutputs/LoggedRobot/GCTimeMS",
    "gc_counts":   "/RealOutputs/LoggedRobot/GCCounts",
    "conduit_cap": "/RealOutputs/Logger/ConduitCaptureMS",
    "conduit_save":"/RealOutputs/Logger/ConduitSaveMS",
    "auto_log":    "/RealOutputs/Logger/AutoLogMS",
    "alert_log":   "/RealOutputs/Logger/AlertLogMS",
    "radio_log":   "/RealOutputs/Logger/RadioLogMS",
    "console":     "/RealOutputs/Logger/ConsoleMS",
    "ds":          "/RealOutputs/Logger/DriverStationMS",
    "entry_update":"/RealOutputs/Logger/EntryUpdateMS",
    "dash_inputs": "/RealOutputs/Logger/DashboardInputsMS",
    "queued":      "/RealOutputs/Logger/QueuedCycles",
}

CORR_KEYS = {
    "can_util":    "/SystemStats/CANBus/Utilization",
    "can_txfull":  "/SystemStats/CANBus/TxFullCount",
    "can_rxerr":   "/SystemStats/CANBus/ReceiveErrorCount",
    "battery":     "/SystemStats/BatteryVoltage",
    "cpu_temp":    "/SystemStats/CPUTempCelsius",
    "tag_count":   "/RealOutputs/Vision/Summary/TagCount",
}


def analyze_match(path, label):
    """Deep overrun analysis for one match."""
    p(f"\n{'='*80}")
    p(f"  DEEP OVERRUN ANALYSIS: {label}")
    p(f"  File: {os.path.basename(path)}")
    p(f"{'='*80}")

    numeric, booleans, strings, string_arrays, doubles_arr = parse_wpilog(path)

    # ── Find match phases ──
    en_pts = booleans.get("/DriverStation/Enabled", [])
    au_pts = booleans.get("/DriverStation/Autonomous", [])
    auto_start, auto_end, teleop_start, teleop_end = find_phases(en_pts, au_pts)

    # ── Extract timing series ──
    timing = {}
    for key, logkey in TIMING_KEYS.items():
        timing[key] = numeric.get(logkey, [])

    corr = {}
    for key, logkey in CORR_KEYS.items():
        corr[key] = numeric.get(logkey, [])

    full_pts = timing["full"]
    user_pts = timing["user"]

    if not full_pts:
        p("  ⚠ No timing data found")
        return None

    # ── Overall timing statistics ──
    full_vals = [v for _, v in full_pts]
    user_vals = [v for _, v in user_pts]
    log_vals = [v for _, v in timing["log_periodic"]]

    total_cycles = len(full_vals)
    overrun_cycles = sum(1 for v in full_vals if v > OVERRUN_THRESHOLD_MS)
    overrun_pct = 100.0 * overrun_cycles / total_cycles if total_cycles else 0

    p(f"\n  ── Overall Timing ──")
    p(f"  Total cycles:      {total_cycles}")
    p(f"  Overrun cycles:    {overrun_cycles} ({overrun_pct:.1f}%)")
    for name, vals in [("FullCycleMS", full_vals), ("UserCodeMS", user_vals), ("LogPeriodicMS", log_vals)]:
        if vals:
            p(f"  {name:20s}: mean={statistics.mean(vals):.2f}  median={statistics.median(vals):.2f}  "
              f"p95={percentile(vals,95):.2f}  p99={percentile(vals,99):.2f}  max={max(vals):.2f}")

    # ── Cycle-time breakdown: what fraction is UserCode vs LogPeriodic? ──
    p(f"\n  ── Cycle Budget Breakdown (averages) ──")
    user_mean = statistics.mean(user_vals) if user_vals else 0
    log_mean = statistics.mean(log_vals) if log_vals else 0
    full_mean = statistics.mean(full_vals) if full_vals else 0
    residual = full_mean - user_mean - log_mean
    p(f"  FullCycleMS avg:       {full_mean:.2f} ms")
    p(f"    ├─ UserCodeMS:       {user_mean:.2f} ms ({100*user_mean/full_mean:.1f}% of total)")
    p(f"    ├─ LogPeriodicMS:    {log_mean:.2f} ms ({100*log_mean/full_mean:.1f}% of total)")
    p(f"    └─ Residual/overlap: {residual:.2f} ms")
    p(f"  Budget remaining:      {LOOP_PERIOD_MS - full_mean:.2f} ms (target: >{0:.1f})")

    # ── Logger component breakdown ──
    logger_components = [
        ("EntryUpdateMS",    timing["entry_update"]),
        ("DashboardInputsMS",timing["dash_inputs"]),
        ("ConduitCaptureMS", timing["conduit_cap"]),
        ("ConduitSaveMS",    timing["conduit_save"]),
        ("AutoLogMS",        timing["auto_log"]),
        ("AlertLogMS",       timing["alert_log"]),
        ("RadioLogMS",       timing["radio_log"]),
        ("ConsoleMS",        timing["console"]),
        ("DriverStationMS",  timing["ds"]),
    ]
    p(f"\n  ── Logger Component Breakdown (ms) ──")
    p(f"  {'Component':24s} {'Mean':>7s} {'Median':>7s} {'P95':>7s} {'P99':>7s} {'Max':>7s}")
    p(f"  {'─'*24} {'─'*7} {'─'*7} {'─'*7} {'─'*7} {'─'*7}")
    logger_total_mean = 0
    for comp_name, comp_pts in logger_components:
        vals = [v for _, v in comp_pts]
        if vals:
            mn = statistics.mean(vals)
            logger_total_mean += mn
            p(f"  {comp_name:24s} {mn:7.3f} {statistics.median(vals):7.3f} "
              f"{percentile(vals,95):7.3f} {percentile(vals,99):7.3f} {max(vals):7.3f}")
        else:
            p(f"  {comp_name:24s}     N/A")
    p(f"  {'─'*24}")
    p(f"  {'Logger Total (sum)':24s} {logger_total_mean:7.3f}")

    # ── GC Analysis ──
    gc_vals = [v for _, v in timing["gc_time"]]
    gc_count_vals = [v for _, v in timing["gc_counts"]]
    p(f"\n  ── Garbage Collection ──")
    if gc_vals:
        gc_nonzero = [v for v in gc_vals if v > 0]
        p(f"  Cycles with GC:       {len(gc_nonzero)} / {len(gc_vals)} ({100*len(gc_nonzero)/len(gc_vals):.1f}%)")
        if gc_nonzero:
            p(f"  GC time when active:  mean={statistics.mean(gc_nonzero):.2f}  max={max(gc_nonzero):.2f} ms")
        if gc_count_vals:
            total_gc_collections = sum(gc_count_vals)
            p(f"  Total GC collections: {total_gc_collections:.0f}")
    else:
        p(f"  No GC data")

    # ── Queue depth analysis ──
    queued_vals = [v for _, v in timing["queued"]]
    if queued_vals:
        p(f"\n  ── Receiver Queue Depth ──")
        p(f"  Mean: {statistics.mean(queued_vals):.1f}  Max: {max(queued_vals):.0f}  "
          f"P99: {percentile(queued_vals,99):.0f}")
        queue_full = sum(1 for v in queued_vals if v >= 100)
        if queue_full:
            p(f"  ⚠ Queue at/near capacity ({queue_full} cycles ≥100)")

    # ── Overrun vs Normal cycle comparison ──
    # Build per-cycle aligned data
    p(f"\n  ── Overrun vs Normal Cycle Comparison ──")
    # Index timing series by timestamp for per-cycle analysis
    cycle_data = []
    full_idx = build_time_index(full_pts)
    for t, full_v in full_idx:
        cd = {"t": t, "full": full_v}
        # Find matching values for other keys (within 1ms)
        for key, pts in timing.items():
            if key == "full":
                continue
            val = nearest_val(pts, t)
            cd[key] = val if val is not None else 0
        for key, pts in corr.items():
            val = nearest_val(pts, t)
            cd[key] = val if val is not None else 0
        cycle_data.append(cd)

    if cycle_data:
        overrun_cycles_data = [c for c in cycle_data if c["full"] > OVERRUN_THRESHOLD_MS]
        normal_cycles_data = [c for c in cycle_data if c["full"] <= OVERRUN_THRESHOLD_MS]

        if overrun_cycles_data and normal_cycles_data:
            p(f"  {'Metric':24s} {'Normal':>10s} {'Overrun':>10s} {'Delta':>10s}")
            p(f"  {'─'*24} {'─'*10} {'─'*10} {'─'*10}")
            compare_keys = ["user", "log_periodic", "gc_time",
                           "conduit_cap", "conduit_save", "auto_log",
                           "entry_update", "dash_inputs", "ds",
                           "alert_log", "console",
                           "can_util", "battery", "tag_count"]
            for key in compare_keys:
                n_vals = [c.get(key, 0) for c in normal_cycles_data if c.get(key) is not None]
                o_vals = [c.get(key, 0) for c in overrun_cycles_data if c.get(key) is not None]
                if n_vals and o_vals:
                    n_mean = statistics.mean(n_vals)
                    o_mean = statistics.mean(o_vals)
                    delta = o_mean - n_mean
                    p(f"  {key:24s} {n_mean:10.3f} {o_mean:10.3f} {delta:+10.3f}")

    # ── Phase-based analysis ──
    p(f"\n  ── Phase-Based Overrun Rates ──")
    phases = []
    if auto_start and auto_end:
        phases.append(("Auto", auto_start, auto_end))
    if teleop_start and teleop_end:
        phases.append(("Teleop", teleop_start, teleop_end))
    # Add disabled phases
    t_min = full_pts[0][0] if full_pts else 0
    if auto_start:
        phases.insert(0, ("Pre-Auto", t_min, auto_start))
    if auto_end and teleop_start:
        phases.append(("Auto→Teleop gap", auto_end, teleop_start))

    for phase_name, t0, t1 in phases:
        phase_full = vals_in_range(full_pts, t0, t1)
        phase_user = vals_in_range(user_pts, t0, t1)
        if phase_full:
            n_over = sum(1 for v in phase_full if v > OVERRUN_THRESHOLD_MS)
            pct = 100 * n_over / len(phase_full)
            user_avg = statistics.mean(phase_user) if phase_user else 0
            p(f"  {phase_name:20s}: {len(phase_full):5d} cycles, {n_over:5d} overruns ({pct:5.1f}%), "
              f"user avg={user_avg:.2f}ms, full avg={statistics.mean(phase_full):.2f}ms")

    # ── Temporal analysis: 5-second windows during teleop ──
    if teleop_start and teleop_end:
        p(f"\n  ── Teleop Overrun Heatmap (5-second windows) ──")
        window_size = 5.0
        t = teleop_start
        worst_windows = []
        while t < teleop_end:
            t_end = min(t + window_size, teleop_end)
            w_full = vals_in_range(full_pts, t, t_end)
            w_user = vals_in_range(user_pts, t, t_end)
            if w_full:
                n_over = sum(1 for v in w_full if v > OVERRUN_THRESHOLD_MS)
                pct = 100 * n_over / len(w_full)
                user_avg = statistics.mean(w_user) if w_user else 0
                # Find what CAN util and tag count look like
                w_can = vals_in_range(corr.get("can_util", []), t, t_end)
                w_tags = vals_in_range(corr.get("tag_count", []), t, t_end)
                can_avg = statistics.mean(w_can) if w_can else 0
                tag_avg = statistics.mean(w_tags) if w_tags else 0
                elapsed = t - teleop_start
                worst_windows.append((elapsed, pct, user_avg, statistics.mean(w_full), can_avg, tag_avg, len(w_full)))
            t += window_size

        # Print worst 10 windows
        worst_windows.sort(key=lambda x: -x[1])
        p(f"  Top 10 worst windows:")
        p(f"  {'T+sec':>6s} {'Overrun%':>8s} {'UserAvg':>8s} {'FullAvg':>8s} {'CAN%':>6s} {'Tags':>5s} {'Cycles':>6s}")
        for w in worst_windows[:10]:
            p(f"  {w[0]:6.0f}s {w[1]:7.1f}% {w[2]:7.2f}ms {w[3]:7.2f}ms {w[4]:5.1f}% {w[5]:5.1f} {w[6]:6d}")

        # Print histogram of overrun rates
        p(f"\n  Overrun rate distribution across windows:")
        buckets = [0]*11  # 0-10%, 10-20%, ..., 90-100%, 100%
        for w in worst_windows:
            idx = min(int(w[1] / 10), 10)
            buckets[idx] += 1
        for i, count in enumerate(buckets):
            lo = i * 10
            hi = lo + 10 if i < 10 else 100
            bar = "█" * count
            p(f"  {lo:3d}-{hi:3d}%: {count:3d} {bar}")

    # ── Correlation analysis ──
    p(f"\n  ── Correlation Analysis (FullCycleMS vs key metrics) ──")
    if len(cycle_data) > 100:
        full_series = [c["full"] for c in cycle_data]
        for corr_key, corr_label in [
            ("user", "UserCodeMS"),
            ("log_periodic", "LogPeriodicMS"),
            ("gc_time", "GCTimeMS"),
            ("can_util", "CAN Bus Utilization"),
            ("battery", "Battery Voltage"),
            ("tag_count", "Vision Tag Count"),
            ("auto_log", "AutoLogMS"),
            ("conduit_save", "ConduitSaveMS"),
            ("entry_update", "EntryUpdateMS"),
        ]:
            corr_series = [c.get(corr_key, 0) for c in cycle_data]
            if any(v != 0 for v in corr_series):
                # Pearson correlation
                n = len(full_series)
                mean_x = statistics.mean(full_series)
                mean_y = statistics.mean(corr_series)
                cov = sum((x - mean_x) * (y - mean_y) for x, y in zip(full_series, corr_series)) / n
                std_x = statistics.stdev(full_series) if len(full_series) > 1 else 1
                std_y = statistics.stdev(corr_series) if len(corr_series) > 1 else 1
                r = cov / (std_x * std_y) if std_x > 0 and std_y > 0 else 0
                strength = "strong" if abs(r) > 0.5 else "moderate" if abs(r) > 0.3 else "weak"
                p(f"  {corr_label:24s}: r = {r:+.4f} ({strength})")

    # ── Active commands during worst overrun bursts ──
    p(f"\n  ── Active Commands During Worst Overruns ──")
    # Find the 20 worst overrun cycles
    worst_cycles = sorted(cycle_data, key=lambda c: -c["full"])[:20]
    if worst_cycles:
        p(f"  20 worst cycles:")
        p(f"  {'Time':>10s} {'FullMS':>8s} {'UserMS':>8s} {'LogMS':>8s} {'GCMS':>6s} {'CAN%':>6s} {'Batt':>5s} {'Tags':>4s}")
        for c in worst_cycles:
            elapsed = c["t"] - (teleop_start or full_pts[0][0])
            p(f"  {elapsed:9.2f}s {c['full']:8.2f} {c.get('user',0):8.2f} "
              f"{c.get('log_periodic',0):8.2f} {c.get('gc_time',0):6.2f} "
              f"{c.get('can_util',0):5.1f}% {c.get('battery',0):5.1f} {c.get('tag_count',0):4.0f}")

    # Check what commands were active during worst overrun windows
    # Commands are logged as boolean keys under /RealOutputs/CommandsAll/ and /RealOutputs/CommandsUnique/
    cmd_keys = [k for k in booleans if "CommandsAll/" in k or "CommandsUnique/" in k]
    if cmd_keys and worst_cycles:
        p(f"\n  Commands active during worst 20 cycles (by frequency):")
        cmd_active_counts = defaultdict(int)
        for c in worst_cycles:
            for cmd_key in cmd_keys:
                cmd_pts = booleans[cmd_key]
                # Find if command was active at this time
                active = False
                for bt, bv in cmd_pts:
                    if bt <= c["t"]:
                        active = bv
                    else:
                        break
                if active:
                    # Clean up key name
                    clean = cmd_key.split("/")[-1]
                    # Remove hash suffix
                    if "_" in clean:
                        name_part = clean.rsplit("_", 1)
                        if len(name_part[1]) <= 8 and all(c in "0123456789abcdef" for c in name_part[1]):
                            clean = name_part[0]
                    cmd_active_counts[clean] += 1

        for cmd, count in sorted(cmd_active_counts.items(), key=lambda x: -x[1])[:15]:
            p(f"    {cmd:40s}: active in {count}/{len(worst_cycles)} worst cycles")

    # ── Overrun streak analysis ──
    p(f"\n  ── Consecutive Overrun Streaks ──")
    streaks = []
    current_streak = 0
    streak_start = 0
    for i, (t, v) in enumerate(full_idx):
        if v > OVERRUN_THRESHOLD_MS:
            if current_streak == 0:
                streak_start = t
            current_streak += 1
        else:
            if current_streak > 0:
                streaks.append((streak_start, current_streak))
            current_streak = 0
    if current_streak > 0:
        streaks.append((streak_start, current_streak))

    if streaks:
        streaks.sort(key=lambda x: -x[1])
        p(f"  Total streaks: {len(streaks)}")
        p(f"  Longest streak: {streaks[0][1]} consecutive cycles at t={streaks[0][0]:.2f}s")
        p(f"  Top 5 streaks:")
        for start_t, length in streaks[:5]:
            elapsed = start_t - (teleop_start or full_pts[0][0])
            p(f"    {length:4d} cycles starting at T+{elapsed:.1f}s")
        # Distribution
        s_lens = [s[1] for s in streaks]
        p(f"  Streak length distribution: mean={statistics.mean(s_lens):.1f}, "
          f"median={statistics.median(s_lens):.0f}, max={max(s_lens)}")

    # ── UserCode spike analysis: what makes UserCodeMS > 18ms? ──
    p(f"\n  ── UserCodeMS Spike Analysis ──")
    user_threshold = 18.0
    spike_cycles = [c for c in cycle_data if c.get("user", 0) > user_threshold]
    normal_user_cycles = [c for c in cycle_data if c.get("user", 0) <= user_threshold]
    if spike_cycles and normal_user_cycles:
        p(f"  Cycles with UserCodeMS > {user_threshold}ms: {len(spike_cycles)} ({100*len(spike_cycles)/len(cycle_data):.1f}%)")
        p(f"\n  Logger overhead comparison (spiky user code vs normal):")
        p(f"  {'Component':24s} {'Normal':>10s} {'Spiky':>10s} {'Delta':>10s}")
        p(f"  {'─'*24} {'─'*10} {'─'*10} {'─'*10}")
        for key in ["auto_log", "conduit_cap", "conduit_save", "entry_update", "dash_inputs",
                     "can_util", "tag_count", "gc_time"]:
            n_vals = [c.get(key, 0) for c in normal_user_cycles]
            s_vals = [c.get(key, 0) for c in spike_cycles]
            if n_vals and s_vals:
                n_m = statistics.mean(n_vals)
                s_m = statistics.mean(s_vals)
                p(f"  {key:24s} {n_m:10.3f} {s_m:10.3f} {s_m-n_m:+10.3f}")

    # ── Return summary for cross-match ──
    result = {
        "label": label,
        "total_cycles": total_cycles,
        "overrun_cycles": overrun_cycles,
        "overrun_pct": overrun_pct,
        "full_mean": full_mean,
        "user_mean": user_mean,
        "log_mean": log_mean,
        "user_p95": percentile(user_vals, 95) if user_vals else 0,
        "user_p99": percentile(user_vals, 99) if user_vals else 0,
        "full_p95": percentile(full_vals, 95) if full_vals else 0,
        "full_p99": percentile(full_vals, 99) if full_vals else 0,
        "gc_pct": (100 * len([v for v in gc_vals if v > 0]) / len(gc_vals)) if gc_vals else 0,
        "gc_mean_when_active": statistics.mean([v for v in gc_vals if v > 0]) if [v for v in gc_vals if v > 0] else 0,
        "can_util_mean": statistics.mean([v for _, v in corr.get("can_util", [])]) if corr.get("can_util") else 0,
        "battery_min": min([v for _, v in corr.get("battery", [])]) if corr.get("battery") else 0,
        "tag_count_mean": statistics.mean([v for _, v in corr.get("tag_count", [])]) if corr.get("tag_count") else 0,
        "auto_overrun_pct": 0,
        "teleop_overrun_pct": 0,
    }
    if auto_start and auto_end:
        af = vals_in_range(full_pts, auto_start, auto_end)
        result["auto_overrun_pct"] = 100 * sum(1 for v in af if v > OVERRUN_THRESHOLD_MS) / len(af) if af else 0
    if teleop_start and teleop_end:
        tf = vals_in_range(full_pts, teleop_start, teleop_end)
        result["teleop_overrun_pct"] = 100 * sum(1 for v in tf if v > OVERRUN_THRESHOLD_MS) / len(tf) if tf else 0

    # Logger component means
    for comp_name, comp_pts in logger_components:
        vals = [v for _, v in comp_pts]
        result[f"logger_{comp_name}"] = statistics.mean(vals) if vals else 0

    # Worst cycle info
    if worst_cycles:
        result["worst_full"] = worst_cycles[0]["full"]
        result["worst_user"] = worst_cycles[0].get("user", 0)
    
    return result


# ══════════════════════════════════════════════════════════════════════════════
# Cross-match analysis
# ══════════════════════════════════════════════════════════════════════════════

def cross_match_analysis(results):
    """Compare overrun patterns across all matches."""
    p(f"\n{'='*80}")
    p(f"  CROSS-MATCH OVERRUN ANALYSIS")
    p(f"{'='*80}")

    # ── Summary table ──
    p(f"\n  ── Comparative Overrun Table ──")
    p(f"  {'Match':10s} {'Cycles':>7s} {'Over%':>6s} {'FullAvg':>8s} {'UserAvg':>8s} {'LogAvg':>7s} "
      f"{'UserP95':>8s} {'UserP99':>8s} {'CAN%':>5s} {'BattMin':>7s} {'GC%':>5s}")
    p(f"  {'─'*10} {'─'*7} {'─'*6} {'─'*8} {'─'*8} {'─'*7} {'─'*8} {'─'*8} {'─'*5} {'─'*7} {'─'*5}")
    for r in results:
        p(f"  {r['label']:10s} {r['total_cycles']:7d} {r['overrun_pct']:5.1f}% "
          f"{r['full_mean']:7.2f}ms {r['user_mean']:7.2f}ms {r['log_mean']:6.2f}ms "
          f"{r['user_p95']:7.2f}ms {r['user_p99']:7.2f}ms "
          f"{r['can_util_mean']:4.1f}% {r['battery_min']:6.1f}V {r['gc_pct']:4.1f}%")

    # ── Auto vs Teleop comparison ──
    p(f"\n  ── Auto vs Teleop Overrun Rates ──")
    p(f"  {'Match':10s} {'Auto%':>7s} {'Teleop%':>8s} {'Delta':>7s}")
    for r in results:
        delta = r["teleop_overrun_pct"] - r["auto_overrun_pct"]
        p(f"  {r['label']:10s} {r['auto_overrun_pct']:6.1f}% {r['teleop_overrun_pct']:7.1f}% {delta:+6.1f}%")

    # ── Logger component comparison ──
    p(f"\n  ── Logger Component Means Across Matches (ms) ──")
    logger_keys = [k for k in results[0] if k.startswith("logger_")]
    header = f"  {'Match':10s}"
    for lk in logger_keys:
        name = lk.replace("logger_", "")[:12]
        header += f" {name:>12s}"
    p(header)
    for r in results:
        row = f"  {r['label']:10s}"
        for lk in logger_keys:
            row += f" {r[lk]:12.3f}"
        p(row)

    # ── Trend analysis ──
    p(f"\n  ── Day-Long Trends ──")
    overrun_pcts = [r["overrun_pct"] for r in results]
    user_means = [r["user_mean"] for r in results]
    can_utils = [r["can_util_mean"] for r in results]
    batt_mins = [r["battery_min"] for r in results]

    p(f"  Overrun rate range:  {min(overrun_pcts):.1f}% – {max(overrun_pcts):.1f}%  "
      f"(span={max(overrun_pcts)-min(overrun_pcts):.1f}%)")
    p(f"  UserCode range:      {min(user_means):.2f} – {max(user_means):.2f} ms  "
      f"(span={max(user_means)-min(user_means):.2f}ms)")
    p(f"  CAN util range:      {min(can_utils):.1f}% – {max(can_utils):.1f}%")
    p(f"  Battery min range:   {min(batt_mins):.1f}V – {max(batt_mins):.1f}V")

    # ── Root cause analysis ──
    p(f"\n  ── ROOT CAUSE ANALYSIS ──")
    
    avg_user = statistics.mean(user_means)
    avg_log = statistics.mean([r["log_mean"] for r in results])
    avg_full = statistics.mean([r["full_mean"] for r in results])
    
    p(f"\n  1. TIMING BUDGET:")
    p(f"     Loop budget: {LOOP_PERIOD_MS:.0f}ms")
    p(f"     Average user code:    {avg_user:.2f}ms ({100*avg_user/LOOP_PERIOD_MS:.0f}% of budget)")
    p(f"     Average log periodic: {avg_log:.2f}ms ({100*avg_log/LOOP_PERIOD_MS:.0f}% of budget)")
    p(f"     Average full cycle:   {avg_full:.2f}ms ({100*avg_full/LOOP_PERIOD_MS:.0f}% of budget)")
    p(f"     Budget headroom:      {LOOP_PERIOD_MS - avg_full:.2f}ms")
    
    p(f"\n  2. USER CODE BREAKDOWN (this is where {100*avg_user/avg_full:.0f}% of time goes):")
    p(f"     The CommandScheduler.run() call encompasses:")
    p(f"     • 4× swerve module IO updates (TalonFX drive + turn + CANcoder = 12 CAN reads)")
    p(f"     • Gyro IO update (Pigeon2 = additional CAN reads)")
    p(f"     • Odometry pose estimation (math)")
    p(f"     • Vision IO updates (4 cameras via PhotonVision = NT reads)")
    p(f"     • Vision pose filtering + poseEstimator.addVisionMeasurement()")
    p(f"     • 7 YAMS mechanism periodic() calls:")
    p(f"       - Flywheel, Hood, Turret, Kicker, Spindexer, IntakePivot, IntakeRollers")
    p(f"       - Each: updateInputs (CAN read) + processInputs (serialization) + updateTelemetry (NT publish)")
    p(f"     • LED subsystem periodic()")
    p(f"     • BatteryLogger current tracking")
    p(f"     • HubShift telemetry recording")
    p(f"     • Active command execute() calls")
    p(f"     Total: ~12+ CAN device reads + 4 vision camera reads + ~14 Logger.recordOutput calls")
    
    p(f"\n  3. CAN BUS ANALYSIS:")
    avg_can = statistics.mean(can_utils)
    p(f"     Average CAN utilization: {avg_can:.1f}%")
    if avg_can > 50:
        p(f"     ⚠ CAN bus is HEAVILY loaded — this is a major contributor to slow IO reads")
    elif avg_can > 30:
        p(f"     ⚠ CAN bus is moderately loaded — IO reads may be delayed by bus contention")
    else:
        p(f"     CAN bus load is manageable")
    
    p(f"\n  4. KEY OBSERVATIONS:")
    # Check if Q39 (lowest overrun) correlates with something
    best = min(results, key=lambda r: r["overrun_pct"])
    worst = max(results, key=lambda r: r["overrun_pct"])
    p(f"     Best match:  {best['label']} ({best['overrun_pct']:.1f}% overrun, "
      f"user={best['user_mean']:.2f}ms, CAN={best['can_util_mean']:.1f}%)")
    p(f"     Worst match: {worst['label']} ({worst['overrun_pct']:.1f}% overrun, "
      f"user={worst['user_mean']:.2f}ms, CAN={worst['can_util_mean']:.1f}%)")
    user_delta = worst["user_mean"] - best["user_mean"]
    p(f"     User code delta: {user_delta:.2f}ms")
    
    p(f"\n  5. ACTIONABLE RECOMMENDATIONS:")
    p(f"     Priority 1 — Reduce User Code Time:")
    p(f"       a) YAMS telemetry verbosity: currently MID. Consider LOW for competition")
    p(f"          (MID publishes voltage + current every cycle; LOW cuts to setpoint/measurement only)")
    p(f"       b) Reduce CAN traffic:")
    p(f"          - Lower update frequency for non-critical motors (kicker, spindexer)")
    p(f"          - Use StatusSignal.waitForAll() with timeout in drive IO to batch CAN reads")
    p(f"       c) Vision: 4 cameras = 4× PhotonVision IO updates per cycle")
    p(f"          - Consider reducing to 2 cameras during match play")
    p(f"          - Or process vision every-other-cycle (Logger.runEveryN(2, ...))")
    p(f"       d) Logger.recordOutput calls: each call serializes + writes to NT")
    p(f"          - Audit and remove redundant/debug outputs during competition")
    p(f"          - Struct arrays (Pose3d[]) are especially expensive to serialize")
    
    p(f"\n     Priority 2 — Reduce Logger Overhead:")
    p(f"       a) Average LogPeriodicMS: {avg_log:.2f}ms")
    avg_auto_log = statistics.mean([r.get("logger_AutoLogMS", 0) for r in results])
    avg_conduit_save = statistics.mean([r.get("logger_ConduitSaveMS", 0) for r in results])
    p(f"       b) AutoLogMS (annotation processing): {avg_auto_log:.3f}ms — ")
    if avg_auto_log > 0.5:
        p(f"          ⚠ Consider reducing @AutoLogOutput annotations")
    else:
        p(f"          OK")
    p(f"       c) ConduitSaveMS: {avg_conduit_save:.3f}ms")
    
    p(f"\n     Priority 3 — Systemic Improvements:")
    p(f"       a) Enable RT thread priority (uncomment Threads.setCurrentThreadPriority in robotPeriodic)")
    p(f"       b) Battery health: worst sag to {min(batt_mins):.1f}V — ensure fresh batteries")
    p(f"       c) Consider CANivore for all TalonFX (if not already) to offload RIO CAN bus")


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    files = sorted(glob.glob(os.path.join(LOG_DIR, "*.wpilog")))
    if not files:
        p(f"No .wpilog files found in {LOG_DIR}")
        return

    p(f"MRT3216 — Deep Loop Overrun Investigation")
    p(f"{'='*60}")
    p(f"Log directory: {LOG_DIR}")
    p(f"Files found:   {len(files)}")
    for f in files:
        p(f"  • {os.path.basename(f)}")

    results = []
    for fpath in files:
        label = extract_label(fpath)
        result = analyze_match(fpath, label)
        if result:
            results.append(result)

    if len(results) > 1:
        cross_match_analysis(results)

    p(f"\n{'='*60}")
    p(f"Analysis complete. Output: {OUTPUT_PATH}")

    _out.close()

if __name__ == "__main__":
    main()
