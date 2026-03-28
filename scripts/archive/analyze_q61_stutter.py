"""Quick Q61 auto stutter root-cause analysis.

Looks at:
 - Drive setpoint vs measured velocity (tracking error)
 - Pose X/Y during auto
 - PP active path name transitions
 - Module turn angles for wheel-fight detection
"""

import struct, sys, os
from collections import defaultdict
from wpiutil.log import DataLogReader

LOG = r"C:\Users\danla\Desktop\Logs Idaho\akit_26-03-28_16-55-43_idbo_q61.wpilog"

def decode_double(raw):
    return struct.unpack('d', struct.pack('q', raw))[0]

reader = DataLogReader(LOG)

# Build entry map
entries = {}
for rec in reader:
    if rec.isStart():
        data_s = rec.getStartData()
        entries[data_s.entry] = (data_s.name, data_s.type)

# Keys we care about
KEYS_OF_INTEREST = [
    # Match phase
    "/DSLog/DSEnabled",
    "/DSLog/DSAuto",
    "/DSLog/DSTeleop",
    # Drive measured (robot-relative chassis speeds)
    "/RealOutputs/SwerveStates/Measured",
    "/RealOutputs/SwerveStates/Setpoints",
    # Drive velocities per module
    "/Drive/Module0/DriveVelocityRadPerSec",
    "/Drive/Module1/DriveVelocityRadPerSec",
    "/Drive/Module2/DriveVelocityRadPerSec",
    "/Drive/Module3/DriveVelocityRadPerSec",
    # Setpoint velocities
    "/Drive/Module0/DriveVelocitySetpointRadPerSec",
    "/Drive/Module1/DriveVelocitySetpointRadPerSec",
    "/Drive/Module2/DriveVelocitySetpointRadPerSec",
    "/Drive/Module3/DriveVelocitySetpointRadPerSec",
    # Turn angles
    "/Drive/Module0/TurnPosition",
    "/Drive/Module1/TurnPosition",
    "/Drive/Module2/TurnPosition",
    "/Drive/Module3/TurnPosition",
    "/Drive/Module0/TurnPositionSetpoint",
    "/Drive/Module1/TurnPositionSetpoint",
    "/Drive/Module2/TurnPositionSetpoint",
    "/Drive/Module3/TurnPositionSetpoint",
    # Odometry pose
    "/Drive/Pose",
    "/RealOutputs/Odometry/Robot",
    # PP path following
    "/PathPlanner/currentPath",
    "/PathPlanner/targetPose",
    "/PathPlanner/currentPose",
    # Commands
    "/RealOutputs/CommandsAll/Repeat(11 - Left Curved Bum Rush)",
    # Loop time
    "/SystemStats/CANBus/Utilization",
    "/RealOutputs/LoggedRobot/FullCycleMS",
    "/RealOutputs/LoggedRobot/LogPeriodicMS",
    "/RealOutputs/LoggedRobot/UserPeriodicMS",
]

# First pass: find which entry IDs match our keys (partial match)
entry_id_to_key = {}
for eid, (name, typ) in entries.items():
    for k in KEYS_OF_INTEREST:
        if k in name or name in k:
            entry_id_to_key[eid] = (name, typ)
            break

print("=== Matched log keys ===")
for eid, (name, typ) in sorted(entry_id_to_key.items(), key=lambda x: x[1][0]):
    print(f"  {typ:20s} {name}")

# Second pass: collect data
data = defaultdict(list)  # key -> [(timestamp, value)]

for rec in reader:
    if rec.isStart() or rec.isFinish() or rec.isControl() or rec.isSetMetadata():
        continue
    eid = rec.getEntry()
    if eid not in entry_id_to_key:
        continue
    name, typ = entry_id_to_key[eid]
    ts = rec.getTimestamp() / 1e6  # microseconds -> seconds

    try:
        if typ == "boolean":
            data[name].append((ts, rec.getBoolean()))
        elif typ == "int64":
            data[name].append((ts, decode_double(rec.getInteger())))
        elif typ == "double":
            data[name].append((ts, rec.getDouble()))
        elif typ == "float":
            data[name].append((ts, rec.getFloat()))
        elif typ == "double[]":
            data[name].append((ts, rec.getDoubleArray()))
        elif typ == "float[]":
            data[name].append((ts, rec.getFloatArray()))
        elif typ == "string":
            data[name].append((ts, rec.getString()))
        elif typ in ("structschema", "struct:Pose2d", "struct:SwerveModuleState[]"):
            # skip binary structs for now
            pass
        elif typ == "raw" or "struct" in typ:
            pass
    except Exception:
        pass

print(f"\n=== Collected {sum(len(v) for v in data.values())} records across {len(data)} keys ===\n")

# Find auto window
enabled_times = data.get("/DSLog/DSEnabled", [])
auto_times = data.get("/DSLog/DSAuto", [])

auto_start = None
auto_end = None
for ts, val in auto_times:
    if val and auto_start is None:
        auto_start = ts
for ts, val in enabled_times:
    if ts > (auto_start or 0) and not val and auto_end is None:
        auto_end = ts

if auto_start is None:
    # Try alternate approach
    print("Could not find auto window from DSLog flags, trying command key...")
    cmd_key = "/RealOutputs/CommandsAll/Repeat(11 - Left Curved Bum Rush)"
    if cmd_key in data:
        for ts, val in data[cmd_key]:
            if val and auto_start is None:
                auto_start = ts
            if auto_start and not val and auto_end is None:
                auto_end = ts

if auto_start is None:
    print("ERROR: Cannot determine auto window")
    sys.exit(1)

print(f"Auto window: {auto_start:.2f}s - {auto_end:.2f}s ({auto_end - auto_start:.1f}s)")

# Helper: filter to auto window with small margin
def in_auto(ts, margin=0.5):
    return (auto_start - margin) <= ts <= (auto_end + margin)

# --- Analysis 1: Drive velocity tracking error ---
print("\n" + "="*70)
print("ANALYSIS 1: Per-module drive velocity tracking error during auto")
print("="*70)

for mod in range(4):
    mod_names = ["FL", "FR", "BL", "BR"]
    meas_key = f"/Drive/Module{mod}/DriveVelocityRadPerSec"
    setp_key = f"/Drive/Module{mod}/DriveVelocitySetpointRadPerSec"

    meas = {ts: v for ts, v in data.get(meas_key, []) if in_auto(ts)}
    setp = {ts: v for ts, v in data.get(setp_key, []) if in_auto(ts)}

    if not meas or not setp:
        print(f"  {mod_names[mod]}: No data")
        continue

    # Align by finding nearest timestamps
    errors = []
    setp_times = sorted(setp.keys())
    for mt, mv in sorted(meas.items()):
        # find closest setpoint
        best_st = min(setp_times, key=lambda st: abs(st - mt))
        if abs(best_st - mt) < 0.05:  # within 50ms
            err = abs(mv - setp[best_st])
            errors.append((mt, err, mv, setp[best_st]))

    if errors:
        avg_err = sum(e[1] for e in errors) / len(errors)
        max_err = max(e[1] for e in errors)
        max_t = [e for e in errors if e[1] == max_err][0][0]
        big_errs = [e for e in errors if e[1] > 10.0]  # > 10 rad/s error
        print(f"  {mod_names[mod]}: avg_err={avg_err:.1f} rad/s, max_err={max_err:.1f} rad/s @ t={max_t:.2f}s, big_errors(>10)={len(big_errs)}/{len(errors)}")

        # Print timeline of worst errors
        if big_errs:
            print(f"    Worst tracking errors:")
            for t, err, mv, sv in sorted(big_errs, key=lambda x: -x[1])[:5]:
                print(f"      t={t:.2f}s  meas={mv:.1f}  setp={sv:.1f}  err={err:.1f} rad/s")

# --- Analysis 2: Turn angle tracking error (wheel fight detection) ---
print("\n" + "="*70)
print("ANALYSIS 2: Turn angle tracking error (wheel fight?)")
print("="*70)

for mod in range(4):
    mod_names = ["FL", "FR", "BL", "BR"]
    meas_key = f"/Drive/Module{mod}/TurnPosition"
    setp_key = f"/Drive/Module{mod}/TurnPositionSetpoint"

    meas = [(ts, v) for ts, v in data.get(meas_key, []) if in_auto(ts)]
    setp = [(ts, v) for ts, v in data.get(setp_key, []) if in_auto(ts)]

    if not meas or not setp:
        print(f"  {mod_names[mod]}: No turn data")
        continue

    # Align
    setp_dict = {ts: v for ts, v in setp}
    setp_times = sorted(setp_dict.keys())
    import math
    errors = []
    for mt, mv in meas:
        best_st = min(setp_times, key=lambda st: abs(st - mt))
        if abs(best_st - mt) < 0.05:
            # Angle error (handle wraparound)
            diff = mv - setp_dict[best_st]
            # Normalize to [-pi, pi]
            diff = (diff + math.pi) % (2*math.pi) - math.pi
            errors.append((mt, abs(diff), mv, setp_dict[best_st]))

    if errors:
        avg_err = sum(e[1] for e in errors) / len(errors)
        max_err = max(e[1] for e in errors)
        big_errs = [e for e in errors if e[1] > 0.1]  # > ~6 degrees
        print(f"  {mod_names[mod]}: avg_err={math.degrees(avg_err):.2f}°, max_err={math.degrees(max_err):.2f}°, big_errors(>6°)={len(big_errs)}/{len(errors)}")

# --- Analysis 3: Velocity setpoint timeline (what PP is commanding) ---
print("\n" + "="*70)
print("ANALYSIS 3: FL drive velocity setpoint timeline during auto")
print("="*70)

setp_key = "/Drive/Module0/DriveVelocitySetpointRadPerSec"
meas_key = "/Drive/Module0/DriveVelocityRadPerSec"
setp_data = [(ts, v) for ts, v in data.get(setp_key, []) if in_auto(ts)]
meas_data = [(ts, v) for ts, v in data.get(meas_key, []) if in_auto(ts)]

if setp_data:
    # Detect sudden setpoint changes (>20 rad/s jump in one cycle)
    jumps = []
    for i in range(1, len(setp_data)):
        dt = setp_data[i][0] - setp_data[i-1][0]
        dv = abs(setp_data[i][1] - setp_data[i-1][1])
        if dv > 20 and dt < 0.05:
            jumps.append((setp_data[i][0], setp_data[i-1][1], setp_data[i][1], dv))

    print(f"  Total setpoint samples in auto: {len(setp_data)}")
    print(f"  Setpoint jumps >20 rad/s: {len(jumps)}")
    for t, v_before, v_after, dv in jumps[:10]:
        print(f"    t={t:.3f}s: {v_before:.1f} -> {v_after:.1f} (Δ={dv:.1f} rad/s)")

    # Detect zero-velocity periods
    zero_periods = []
    in_zero = False
    zero_start = 0
    for ts, v in setp_data:
        if abs(v) < 0.5:
            if not in_zero:
                in_zero = True
                zero_start = ts
        else:
            if in_zero:
                zero_periods.append((zero_start, ts, ts - zero_start))
                in_zero = False
    if in_zero:
        zero_periods.append((zero_start, setp_data[-1][0], setp_data[-1][0] - zero_start))

    print(f"\n  Zero-velocity setpoint periods (robot told to stop):")
    for start, end, dur in zero_periods:
        print(f"    {start:.2f}s - {end:.2f}s ({dur:.2f}s)")

# --- Analysis 4: Loop timing during auto ---
print("\n" + "="*70)
print("ANALYSIS 4: Loop timing during auto")
print("="*70)

for timing_key in ["/RealOutputs/LoggedRobot/FullCycleMS",
                    "/RealOutputs/LoggedRobot/UserPeriodicMS"]:
    timing = [(ts, v) for ts, v in data.get(timing_key, []) if in_auto(ts)]
    if timing:
        vals = [v for _, v in timing]
        label = timing_key.split("/")[-1]
        avg = sum(vals)/len(vals)
        p50 = sorted(vals)[len(vals)//2]
        p95 = sorted(vals)[int(len(vals)*0.95)]
        p99 = sorted(vals)[int(len(vals)*0.99)]
        mx = max(vals)
        over20 = sum(1 for v in vals if v > 20)
        over30 = sum(1 for v in vals if v > 30)
        print(f"  {label}: avg={avg:.1f}ms p50={p50:.1f}ms p95={p95:.1f}ms p99={p99:.1f}ms max={mx:.1f}ms >20ms={over20}/{len(vals)} >30ms={over30}/{len(vals)}")

        # Show worst spikes
        worst = sorted(timing, key=lambda x: -x[1])[:5]
        print(f"    Worst: ", end="")
        print(", ".join(f"t={t:.2f}s:{v:.0f}ms" for t,v in worst))

# --- Analysis 5: Setpoint vs Measured full timeline ---
print("\n" + "="*70)
print("ANALYSIS 5: FL setpoint vs measured - sampled every 0.5s through auto")
print("="*70)

if setp_data and meas_data:
    meas_dict = {ts: v for ts, v in meas_data}
    meas_times = sorted(meas_dict.keys())

    t = auto_start
    while t <= auto_end:
        # Find closest setpoint and measured
        closest_setp = min(setp_data, key=lambda x: abs(x[0] - t))
        closest_meas = min(meas_data, key=lambda x: abs(x[0] - t))
        if abs(closest_setp[0] - t) < 0.1 and abs(closest_meas[0] - t) < 0.1:
            err = closest_meas[1] - closest_setp[1]
            print(f"  t={t:.1f}s  setp={closest_setp[1]:7.1f}  meas={closest_meas[1]:7.1f}  err={err:+6.1f} rad/s")
        t += 0.5

print("\nDone.")
