"""
MRT3216 — Loop Timing Analysis for a single match log.
Analyzes loop period, overruns, timing spikes, and correlates with robot state.
Usage: python analyze_loop_timing.py <path_to_wpilog>
"""

import os, sys, struct, statistics
from collections import defaultdict
from wpiutil.log import DataLogReader

if len(sys.argv) < 2:
    print("Usage: python analyze_loop_timing.py <path_to_wpilog>")
    sys.exit(1)

LOG_PATH = sys.argv[1]
NOMINAL_PERIOD_MS = 20.0
OVERRUN_THRESHOLD_MS = 22.0  # >10% over nominal
SPIKE_THRESHOLD_MS = 40.0    # severe spike
GC_THRESHOLD_MS = 50.0       # likely GC pause

print(f"Reading {os.path.basename(LOG_PATH)} ({os.path.getsize(LOG_PATH)/1e6:.1f} MB)...")

# ── Pass 1: Discover keys ──────────────────────────────────────────────────
entry_map = {}   # entry_id -> name
type_map = {}    # entry_id -> type

for rec in DataLogReader(LOG_PATH):
    if rec.isStart():
        d = rec.getStartData()
        entry_map[d.entry] = d.name
        type_map[d.entry] = d.type

# Find relevant keys
loop_period_key = None
enabled_key = None
auto_key = None
battery_key = None
can_util_key = None
watchdog_key = None
fpga_key = None

for eid, name in entry_map.items():
    nl = name.lower()
    if "loopperiod" in nl or "loop_period" in nl or name == "/RealOutputs/LoggedRobot/FullCycleMS":
        loop_period_key = eid
        print(f"  Loop period key: {name} (type={type_map[eid]})")
    elif name == "/DSConnected" or name == "/RealOutputs/DSConnected":
        pass  # skip
    elif name == "/RealOutputs/LoggedRobot/RealPeriodSeconds" or "realtimestamp" in nl:
        if loop_period_key is None:
            loop_period_key = eid
            print(f"  Loop period key (alt): {name} (type={type_map[eid]})")
    elif "enabled" in nl and "auto" not in nl and "fms" not in nl and "nt" not in nl and "stator" not in nl:
        if enabled_key is None:
            enabled_key = eid
            print(f"  Enabled key: {name}")
    elif name in ("/FMSInfo/IsAutonomous", "/DSLog/IsAutonomous"):
        auto_key = eid
        print(f"  Auto key: {name}")
    elif "batteryvoltage" in nl or "voltage/value" in nl:
        if battery_key is None:
            battery_key = eid
            print(f"  Battery key: {name}")
    elif "canutil" in nl or "canutilization" in nl or name == "/SystemStats/CANBus/Utilization":
        can_util_key = eid
        print(f"  CAN util key: {name}")

# Try harder to find loop period
if loop_period_key is None:
    for eid, name in entry_map.items():
        if "FullCycle" in name or "cycleTime" in name.lower() or "period" in name.lower():
            loop_period_key = eid
            print(f"  Loop period key (search): {name} (type={type_map[eid]})")
            break

# List all LoggedRobot keys for debugging
print("\n  LoggedRobot keys found:")
for eid, name in sorted(entry_map.items(), key=lambda x: x[1]):
    if "LoggedRobot" in name or "loopperiod" in name.lower():
        print(f"    {name} (type={type_map[eid]})")

# ── Pass 2: Extract data ───────────────────────────────────────────────────
loop_times = []       # (t_sec, period_ms)
enabled_times = []    # (t_sec, bool)
auto_times = []       # (t_sec, bool)
battery_times = []    # (t_sec, volts)
can_util_times = []   # (t_sec, pct)

# Also collect raw timestamps of ALL records to compute inter-record gaps
all_timestamps = []

def decode_double(rec):
    """AdvantageKit encodes doubles as int64 bit patterns."""
    try:
        return rec.getDouble()
    except:
        try:
            raw = rec.getInteger()
            return struct.unpack('d', struct.pack('q', raw))[0]
        except:
            return None

for rec in DataLogReader(LOG_PATH):
    if rec.isStart() or rec.isFinish() or rec.isControl() or rec.isSetMetadata():
        continue

    eid = rec.getEntry()
    if eid not in entry_map:
        continue

    t = rec.getTimestamp() / 1e6

    if eid == loop_period_key:
        val = decode_double(rec)
        if val is not None:
            # Could be in seconds or milliseconds
            if val < 1.0:  # likely seconds
                loop_times.append((t, val * 1000.0))
            else:
                loop_times.append((t, val))
    elif eid == enabled_key:
        try:
            enabled_times.append((t, rec.getBoolean()))
        except:
            pass
    elif eid == auto_key:
        try:
            auto_times.append((t, rec.getBoolean()))
        except:
            pass
    elif eid == battery_key:
        val = decode_double(rec)
        if val is not None:
            battery_times.append((t, val))
    elif eid == can_util_key:
        try:
            can_util_times.append((t, rec.getFloat()))
        except:
            val = decode_double(rec)
            if val is not None:
                can_util_times.append((t, val))

print(f"\nCollected {len(loop_times)} loop period samples")
print(f"Collected {len(battery_times)} battery voltage samples")
print(f"Collected {len(can_util_times)} CAN util samples")

if not loop_times:
    print("\n⚠️  No loop period data found! Listing all available numeric keys:")
    for eid, name in sorted(entry_map.items(), key=lambda x: x[1]):
        print(f"  {name} (type={type_map[eid]})")
    sys.exit(1)

# ── Determine match phases ─────────────────────────────────────────────────
# Find auto/teleop/disabled boundaries from enabled + auto signals
def find_phase_boundaries():
    """Return list of (start, end, phase_name) tuples."""
    phases = []
    # Combine enabled and auto into state at each timestamp
    events = []
    for t, v in enabled_times:
        events.append((t, 'enabled', v))
    for t, v in auto_times:
        events.append((t, 'auto', v))
    events.sort()

    enabled = False
    auto = False
    phase_start = None
    last_phase = None

    for t, kind, val in events:
        if kind == 'enabled':
            enabled = val
        elif kind == 'auto':
            auto = val

        if enabled and auto:
            phase = 'AUTO'
        elif enabled:
            phase = 'TELEOP'
        else:
            phase = 'DISABLED'

        if phase != last_phase:
            if last_phase is not None and phase_start is not None:
                phases.append((phase_start, t, last_phase))
            phase_start = t
            last_phase = phase

    if last_phase is not None and phase_start is not None:
        phases.append((phase_start, loop_times[-1][0] if loop_times else phase_start + 1, last_phase))

    return phases

phases = find_phase_boundaries()
print(f"\nMatch phases detected:")
for start, end, name in phases:
    dur = end - start
    print(f"  {name:10s}: {start:7.1f}s – {end:7.1f}s  ({dur:.1f}s)")

# ── Analysis ────────────────────────────────────────────────────────────────
def analyze_phase(phase_name, t_start, t_end):
    """Analyze loop timing within a specific phase."""
    pts = [(t, p) for t, p in loop_times if t_start <= t <= t_end]
    if not pts:
        return None

    periods = [p for _, p in pts]
    overruns = [(t, p) for t, p in pts if p > OVERRUN_THRESHOLD_MS]
    spikes = [(t, p) for t, p in pts if p > SPIKE_THRESHOLD_MS]
    gc_spikes = [(t, p) for t, p in pts if p > GC_THRESHOLD_MS]

    # Find clusters of overruns (within 2s of each other)
    overrun_clusters = []
    if overruns:
        cluster = [overruns[0]]
        for i in range(1, len(overruns)):
            if overruns[i][0] - cluster[-1][0] < 2.0:
                cluster.append(overruns[i])
            else:
                if len(cluster) >= 3:
                    overrun_clusters.append(cluster)
                cluster = [overruns[i]]
        if len(cluster) >= 3:
            overrun_clusters.append(cluster)

    # Battery correlation: check voltage during spikes
    spike_voltages = []
    for st, sp in spikes:
        # Find closest battery reading
        closest = min(battery_times, key=lambda x: abs(x[0] - st), default=None)
        if closest and abs(closest[0] - st) < 1.0:
            spike_voltages.append(closest[1])

    return {
        'phase': phase_name,
        'duration': t_end - t_start,
        'samples': len(pts),
        'mean': statistics.mean(periods),
        'median': statistics.median(periods),
        'stdev': statistics.stdev(periods) if len(periods) > 1 else 0,
        'p95': sorted(periods)[int(0.95 * len(periods))] if periods else 0,
        'p99': sorted(periods)[int(0.99 * len(periods))] if periods else 0,
        'max': max(periods),
        'min': min(periods),
        'overrun_count': len(overruns),
        'overrun_pct': 100.0 * len(overruns) / len(pts),
        'spike_count': len(spikes),
        'gc_spike_count': len(gc_spikes),
        'overrun_clusters': overrun_clusters,
        'worst_5': sorted(pts, key=lambda x: -x[1])[:5],
        'spike_voltages': spike_voltages,
    }


print("\n" + "=" * 80)
print("  LOOP TIMING ANALYSIS")
print("=" * 80)

# Overall analysis
all_periods = [p for _, p in loop_times]
print(f"\n  OVERALL ({len(all_periods)} samples, {loop_times[-1][0] - loop_times[0][0]:.1f}s)")
print(f"    Mean:   {statistics.mean(all_periods):.2f} ms")
print(f"    Median: {statistics.median(all_periods):.2f} ms")
print(f"    Stdev:  {statistics.stdev(all_periods):.2f} ms")
print(f"    P95:    {sorted(all_periods)[int(0.95*len(all_periods))]:.2f} ms")
print(f"    P99:    {sorted(all_periods)[int(0.99*len(all_periods))]:.2f} ms")
print(f"    Max:    {max(all_periods):.2f} ms")
print(f"    Min:    {min(all_periods):.2f} ms")
overrun_total = sum(1 for p in all_periods if p > OVERRUN_THRESHOLD_MS)
print(f"    Overruns (>{OVERRUN_THRESHOLD_MS}ms): {overrun_total} ({100*overrun_total/len(all_periods):.1f}%)")

# Per-phase analysis
for start, end, name in phases:
    if name == 'DISABLED':
        continue
    result = analyze_phase(name, start, end)
    if result is None:
        continue

    print(f"\n  {result['phase']} ({result['duration']:.1f}s, {result['samples']} samples)")
    print(f"    Mean:     {result['mean']:.2f} ms")
    print(f"    Median:   {result['median']:.2f} ms")
    print(f"    Stdev:    {result['stdev']:.2f} ms")
    print(f"    P95:      {result['p95']:.2f} ms")
    print(f"    P99:      {result['p99']:.2f} ms")
    print(f"    Max:      {result['max']:.2f} ms")
    print(f"    Overruns: {result['overrun_count']} ({result['overrun_pct']:.1f}%)")
    print(f"    Spikes (>{SPIKE_THRESHOLD_MS}ms): {result['spike_count']}")
    print(f"    GC-class (>{GC_THRESHOLD_MS}ms):  {result['gc_spike_count']}")

    print(f"    Worst 5 cycles:")
    for t, p in result['worst_5']:
        phase_offset = t - start
        print(f"      t={t:.2f}s (+{phase_offset:.2f}s into {name}): {p:.1f} ms")

    if result['spike_voltages']:
        print(f"    Battery during spikes: min={min(result['spike_voltages']):.2f}V, "
              f"avg={statistics.mean(result['spike_voltages']):.2f}V")

    if result['overrun_clusters']:
        print(f"    Overrun clusters (≥3 consecutive):")
        for i, cluster in enumerate(result['overrun_clusters'][:10]):
            t0, t1 = cluster[0][0], cluster[-1][0]
            avg_p = statistics.mean([p for _, p in cluster])
            print(f"      Cluster {i+1}: t={t0:.1f}–{t1:.1f}s ({len(cluster)} overruns, "
                  f"avg={avg_p:.1f}ms, max={max(p for _,p in cluster):.1f}ms)")

# ── Timing distribution histogram ──────────────────────────────────────────
print(f"\n{'='*80}")
print(f"  TIMING DISTRIBUTION (in-match only)")
print(f"{'='*80}")

# Filter to enabled-only periods
match_periods = []
for t, p in loop_times:
    for start, end, name in phases:
        if start <= t <= end and name != 'DISABLED':
            match_periods.append(p)
            break

if match_periods:
    bins = [0, 15, 18, 20, 22, 25, 30, 40, 50, 75, 100, 200, float('inf')]
    labels = ["<15", "15-18", "18-20", "20-22", "22-25", "25-30", "30-40", "40-50",
              "50-75", "75-100", "100-200", ">200"]
    counts = [0] * len(labels)
    for p in match_periods:
        for i in range(len(bins) - 1):
            if bins[i] <= p < bins[i+1]:
                counts[i] += 1
                break

    max_count = max(counts) if counts else 1
    print(f"\n  {'Range':>10s}  {'Count':>6s}  {'%':>6s}  Bar")
    print(f"  {'─'*10}  {'─'*6}  {'─'*6}  {'─'*40}")
    for label, count in zip(labels, counts):
        pct = 100 * count / len(match_periods)
        bar = '█' * int(40 * count / max_count) if count > 0 else ''
        flag = ' ⚠️' if label in ('>200', '100-200', '75-100') and count > 0 else ''
        print(f"  {label:>10s}  {count:6d}  {pct:5.1f}%  {bar}{flag}")

# ── Battery voltage during match ────────────────────────────────────────────
if battery_times:
    print(f"\n{'='*80}")
    print(f"  BATTERY VOLTAGE")
    print(f"{'='*80}")

    match_volts = []
    for t, v in battery_times:
        for start, end, name in phases:
            if start <= t <= end and name != 'DISABLED':
                match_volts.append(v)
                break

    if match_volts:
        print(f"  In-match: min={min(match_volts):.2f}V, avg={statistics.mean(match_volts):.2f}V, "
              f"max={max(match_volts):.2f}V")
        low_count = sum(1 for v in match_volts if v < 7.0)
        brownout_count = sum(1 for v in match_volts if v < 6.8)
        print(f"  Below 7.0V: {low_count} samples ({100*low_count/len(match_volts):.1f}%)")
        print(f"  Below 6.8V (brownout): {brownout_count} samples")

# ── CAN utilization during match ────────────────────────────────────────────
if can_util_times:
    print(f"\n{'='*80}")
    print(f"  CAN UTILIZATION")
    print(f"{'='*80}")

    match_can = []
    for t, v in can_util_times:
        for start, end, name in phases:
            if start <= t <= end and name != 'DISABLED':
                match_can.append(v)
                break

    if match_can:
        print(f"  In-match: min={min(match_can):.1f}%, avg={statistics.mean(match_can):.1f}%, "
              f"max={max(match_can):.1f}%")

# ── Timeline: overruns over time ────────────────────────────────────────────
print(f"\n{'='*80}")
print(f"  OVERRUN TIMELINE (>{OVERRUN_THRESHOLD_MS}ms cycles, 5s windows)")
print(f"{'='*80}")

# Bucket overruns into 5s windows
if phases:
    match_start = min(s for s, e, n in phases if n != 'DISABLED')
    match_end = max(e for s, e, n in phases if n != 'DISABLED')

    window = 5.0
    t = match_start
    print(f"\n  {'Window':>12s}  {'Phase':>8s}  {'Overruns':>8s}  {'Worst':>8s}  Visual")
    print(f"  {'─'*12}  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*30}")

    while t < match_end:
        t_end = t + window
        window_pts = [(ts, p) for ts, p in loop_times if t <= ts < t_end and p > OVERRUN_THRESHOLD_MS]
        all_window = [(ts, p) for ts, p in loop_times if t <= ts < t_end]

        # Determine phase
        phase = "?"
        for ps, pe, pn in phases:
            if ps <= t + window/2 <= pe:
                phase = pn
                break

        if window_pts and phase != 'DISABLED':
            worst = max(p for _, p in window_pts)
            bar_len = min(30, len(window_pts))
            color = '🟥' if worst > GC_THRESHOLD_MS else ('🟧' if worst > SPIKE_THRESHOLD_MS else '🟨')
            bar = color * min(10, len(window_pts))
            print(f"  {t:6.1f}-{t_end:5.1f}s  {phase:>8s}  {len(window_pts):8d}  {worst:7.1f}ms  {bar}")

        t += window

# ── Summary diagnosis ───────────────────────────────────────────────────────
print(f"\n{'='*80}")
print(f"  DIAGNOSIS")
print(f"{'='*80}")

all_match_periods = []
for t, p in loop_times:
    for start, end, name in phases:
        if start <= t <= end and name != 'DISABLED':
            all_match_periods.append(p)
            break

if all_match_periods:
    mean_p = statistics.mean(all_match_periods)
    overrun_pct = 100 * sum(1 for p in all_match_periods if p > OVERRUN_THRESHOLD_MS) / len(all_match_periods)
    spike_pct = 100 * sum(1 for p in all_match_periods if p > SPIKE_THRESHOLD_MS) / len(all_match_periods)

    if overrun_pct > 30:
        print(f"  🔴 SEVERE: {overrun_pct:.1f}% of match cycles exceed {OVERRUN_THRESHOLD_MS}ms")
        print(f"     This means the robot is running slower than real-time for ~1/3 of the match.")
        print(f"     Subsystem code or logging is likely too heavy for the 20ms budget.")
    elif overrun_pct > 15:
        print(f"  🟠 WARNING: {overrun_pct:.1f}% of match cycles exceed {OVERRUN_THRESHOLD_MS}ms")
        print(f"     Significant overruns — may cause jerky motion and degraded tracking.")
    elif overrun_pct > 5:
        print(f"  🟡 MODERATE: {overrun_pct:.1f}% of match cycles exceed {OVERRUN_THRESHOLD_MS}ms")
    else:
        print(f"  🟢 GOOD: Only {overrun_pct:.1f}% of match cycles exceed {OVERRUN_THRESHOLD_MS}ms")

    if spike_pct > 5:
        print(f"  🔴 {spike_pct:.1f}% of cycles are severe spikes (>{SPIKE_THRESHOLD_MS}ms)")
        print(f"     These cause visible jerks and can destabilize path following.")

    gc_count = sum(1 for p in all_match_periods if p > GC_THRESHOLD_MS)
    if gc_count > 10:
        print(f"  🟠 {gc_count} GC-class spikes (>{GC_THRESHOLD_MS}ms) detected")
        print(f"     Consider tuning JVM GC or reducing object allocation.")
    elif gc_count > 0:
        print(f"  🟡 {gc_count} GC-class spikes (>{GC_THRESHOLD_MS}ms) — minor but watch for growth")

    if mean_p > 22:
        print(f"  🔴 Mean loop period ({mean_p:.1f}ms) exceeds 20ms — robot is consistently behind real-time")
    elif mean_p > 20.5:
        print(f"  🟡 Mean loop period ({mean_p:.1f}ms) slightly above nominal")

print("\nDone.")
