"""Q67 loop timing breakdown using AdvantageKit's built-in timing keys."""
import struct, statistics, sys
from wpiutil.log import DataLogReader

LOG = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\danla\Desktop\Logs Idaho\akit_26-03-28_17-53-34_idbo_q67.wpilog"

entry_map = {}
for rec in DataLogReader(LOG):
    if rec.isStart():
        d = rec.getStartData()
        entry_map[d.entry] = d.name

# Find match boundaries
ds_enabled = []
ds_auto = []
timing_keys = {
    "FullCycle": "/RealOutputs/LoggedRobot/FullCycleMS",
    "UserCode": "/RealOutputs/LoggedRobot/UserCodeMS",
    "LogPeriodic": "/RealOutputs/LoggedRobot/LogPeriodicMS",
    "GCTime": "/RealOutputs/LoggedRobot/GCTimeMS",
    "GCCounts": "/RealOutputs/LoggedRobot/GCCounts",
    "AutoLog": "/RealOutputs/Logger/AutoLogMS",
    "CondCapture": "/RealOutputs/Logger/ConduitCaptureMS",
    "CondSave": "/RealOutputs/Logger/ConduitSaveMS",
    "Console": "/RealOutputs/Logger/ConsoleMS",
    "DashInputs": "/RealOutputs/Logger/DashboardInputsMS",
    "DS": "/RealOutputs/Logger/DriverStationMS",
    "EntryUpdate": "/RealOutputs/Logger/EntryUpdateMS",
    "AlertLog": "/RealOutputs/Logger/AlertLogMS",
    "RadioLog": "/RealOutputs/Logger/RadioLogMS",
}

key_eids = {}
for short, full in timing_keys.items():
    for eid, name in entry_map.items():
        if name == full:
            key_eids[eid] = short

# Also find enabled/auto eids
en_eid = au_eid = None
for eid, name in entry_map.items():
    if name == "/DriverStation/Enabled":
        en_eid = eid
    elif name == "/DriverStation/Autonomous":
        au_eid = eid

# Extract everything in one pass
data = {k: [] for k in timing_keys}

for rec in DataLogReader(LOG):
    if rec.isStart() or rec.isFinish() or rec.isControl() or rec.isSetMetadata():
        continue
    eid = rec.getEntry()
    if eid not in entry_map:
        continue
    t = rec.getTimestamp() / 1e6

    if eid == en_eid:
        ds_enabled.append((t, rec.getBoolean()))
    elif eid == au_eid:
        ds_auto.append((t, rec.getBoolean()))
    elif eid in key_eids:
        try:
            raw = rec.getInteger()
            val = struct.unpack("d", struct.pack("q", raw))[0]
        except Exception:
            try:
                val = rec.getDouble()
            except Exception:
                continue
        if val < 0 or val > 50000:
            continue
        data[key_eids[eid]].append((t, val))

# Build phases
enabled = False
auto = False
events = [(t, "e", v) for t, v in ds_enabled] + [(t, "a", v) for t, v in ds_auto]
events.sort()
phases = []
phase_start = None
last_phase = None
for t, kind, val in events:
    if kind == "e":
        enabled = val
    elif kind == "a":
        auto = val
    if enabled and auto:
        phase = "AUTO"
    elif enabled:
        phase = "TELEOP"
    else:
        phase = "DISABLED"
    if phase != last_phase:
        if last_phase and phase_start:
            phases.append((phase_start, t, last_phase))
        phase_start = t
        last_phase = phase
if last_phase and phase_start:
    phases.append((phase_start, 9999, last_phase))

print("Match phases:")
for s, e, n in phases:
    if n != "DISABLED":
        print(f"  {n}: {s:.1f}s - {e:.1f}s ({e-s:.1f}s)")

# Filter data to in-match only
def in_match(t):
    for s, e, n in phases:
        if s <= t <= e and n != "DISABLED":
            return True
    return False

for phase_name in ["AUTO", "TELEOP"]:
    phase_ranges = [(s, e) for s, e, n in phases if n == phase_name]
    if not phase_ranges:
        continue

    print(f"\n{'='*85}")
    print(f"  {phase_name} TIMING BREAKDOWN")
    print(f"{'='*85}")

    header = (
        f"{'Component':>15s} | {'Mean':>7s} | {'Median':>7s} | "
        f"{'P95':>7s} | {'P99':>7s} | {'Max':>8s} | {'N':>6s}"
    )
    print(header)
    print("-" * 85)

    results = []
    for short in timing_keys:
        pts = []
        for t, v in data[short]:
            for ps, pe in phase_ranges:
                if ps <= t <= pe:
                    pts.append(v)
                    break
        if not pts:
            continue
        n = len(pts)
        results.append((
            short,
            statistics.mean(pts),
            statistics.median(pts),
            sorted(pts)[int(0.95 * n)],
            sorted(pts)[int(0.99 * n)],
            max(pts),
            n,
        ))

    results.sort(key=lambda x: -x[1])
    for short, mean, med, p95, p99, mx, n in results:
        print(
            f"{short:>15s} | {mean:6.2f}ms | {med:6.2f}ms | "
            f"{p95:6.2f}ms | {p99:6.2f}ms | {mx:7.2f}ms | {n:6d}"
        )

    # Summary
    fc_vals = []
    uc_vals = []
    lp_vals = []
    for t, v in data["FullCycle"]:
        for ps, pe in phase_ranges:
            if ps <= t <= pe:
                fc_vals.append(v)
                break
    for t, v in data["UserCode"]:
        for ps, pe in phase_ranges:
            if ps <= t <= pe:
                uc_vals.append(v)
                break
    for t, v in data["LogPeriodic"]:
        for ps, pe in phase_ranges:
            if ps <= t <= pe:
                lp_vals.append(v)
                break

    if fc_vals and uc_vals and lp_vals:
        fc_m = statistics.mean(fc_vals)
        uc_m = statistics.mean(uc_vals)
        lp_m = statistics.mean(lp_vals)
        print()
        print(f"  FullCycle mean:      {fc_m:.2f}ms")
        print(f"  UserCode mean:       {uc_m:.2f}ms  ({100*uc_m/fc_m:.0f}% of cycle)")
        print(f"  LogPeriodic mean:    {lp_m:.2f}ms  ({100*lp_m/fc_m:.0f}% of cycle)")
        print(f"  Overhead (FC-UC-LP): {fc_m-uc_m-lp_m:.2f}ms  ({100*(fc_m-uc_m-lp_m)/fc_m:.0f}% of cycle)")

# GC analysis
print(f"\n{'='*85}")
print(f"  GARBAGE COLLECTION")
print(f"{'='*85}")
gc_c = [v for t, v in data["GCCounts"] if in_match(t)]
gc_t = [v for t, v in data["GCTime"] if in_match(t)]
if gc_c:
    print(f"  Total GC collections in match: {max(gc_c) - min(gc_c):.0f}")
    print(f"  GC time per cycle: mean={statistics.mean(gc_t):.3f}ms, max={max(gc_t):.2f}ms")
    print(f"  Cycles with >5ms GC: {sum(1 for v in gc_t if v > 5)}")
    print(f"  Cycles with >10ms GC: {sum(1 for v in gc_t if v > 10)}")
    print(f"  Cycles with >20ms GC: {sum(1 for v in gc_t if v > 20)}")

# Logger sub-breakdown
print(f"\n{'='*85}")
print(f"  LOGGER SUB-BREAKDOWN (mean ms)")
print(f"{'='*85}")
logger_keys = ["AutoLog", "CondCapture", "CondSave", "Console", "DashInputs", "DS", "EntryUpdate", "AlertLog", "RadioLog"]
total_logger = 0
for k in logger_keys:
    vals = [v for t, v in data[k] if in_match(t)]
    if vals:
        m = statistics.mean(vals)
        total_logger += m
        p95 = sorted(vals)[int(0.95 * len(vals))]
        print(f"  {k:>15s}: {m:6.3f}ms  (P95={p95:.3f}ms, max={max(vals):.2f}ms)")
print(f"  {'TOTAL':>15s}: {total_logger:6.3f}ms")

# Now check what percentage of UserCode is accounted for by what we can see
print(f"\n{'='*85}")
print(f"  WHERE IS THE TIME GOING?")
print(f"{'='*85}")
uc_m = statistics.mean([v for t, v in data["UserCode"] if in_match(t)]) if data["UserCode"] else 0
lp_m = statistics.mean([v for t, v in data["LogPeriodic"] if in_match(t)]) if data["LogPeriodic"] else 0
fc_m = statistics.mean([v for t, v in data["FullCycle"] if in_match(t)]) if data["FullCycle"] else 0
gc_m = statistics.mean([v for t, v in data["GCTime"] if in_match(t)]) if data["GCTime"] else 0

print(f"  Full cycle:    {fc_m:.2f}ms")
print(f"  = UserCode:    {uc_m:.2f}ms  (subsystem periodic + command scheduler)")
print(f"  + LogPeriodic: {lp_m:.2f}ms  (AdvantageKit logging)")
print(f"  + GC overhead: {gc_m:.3f}ms")
print(f"  + Other:       {fc_m - uc_m - lp_m:.2f}ms  (HAL, odometry thread, etc.)")
print(f"  Budget:        20.00ms")
print(f"  Over budget:   {fc_m - 20:.2f}ms")
