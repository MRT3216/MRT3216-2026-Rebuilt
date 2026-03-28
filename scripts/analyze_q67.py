"""Q67 auto + teleop stutter analysis — adapted from Q61 v2 script."""

import struct as st
import sys, math
from collections import defaultdict
from wpiutil.log import DataLogReader

LOG = r"C:\Users\danla\Desktop\Logs Idaho\akit_26-03-28_17-53-34_idbo_q67.wpilog"

reader = DataLogReader(LOG)

entries = {}
for rec in reader:
    if rec.isStart():
        d = rec.getStartData()
        entries[d.entry] = (d.name, d.type)

# Print all command keys so we know the auto name
print("=== Command keys ===")
for eid, (name, typ) in entries.items():
    if "CommandsAll" in name or "CommandsUnique" in name:
        print(f"  {typ:10s} {name}")

# Keys we want
KEYS = {
    "/Drive/Module0/DriveVelocityRadPerSec": "double",
    "/Drive/Module1/DriveVelocityRadPerSec": "double",
    "/Drive/Module2/DriveVelocityRadPerSec": "double",
    "/Drive/Module3/DriveVelocityRadPerSec": "double",
    "/Drive/Module0/DriveAppliedVolts": "double",
    "/Drive/Module1/DriveAppliedVolts": "double",
    "/Drive/Module2/DriveAppliedVolts": "double",
    "/Drive/Module3/DriveAppliedVolts": "double",
    "/RealOutputs/SwerveStates/Setpoints": "struct:SwerveModuleState[]",
    "/RealOutputs/SwerveStates/Measured": "struct:SwerveModuleState[]",
    "/RealOutputs/SwerveStates/SetpointsOptimized": "struct:SwerveModuleState[]",
    "/RealOutputs/SwerveChassisSpeeds/Setpoints": "struct:ChassisSpeeds",
    "/RealOutputs/SwerveChassisSpeeds/Measured": "struct:ChassisSpeeds",
    "/RealOutputs/LoggedRobot/FullCycleMS": "double",
    "/SystemStats/CANBus/Utilization": "float",
}

# Dynamically add any Repeat(...) command key
for eid, (name, typ) in entries.items():
    if "CommandsAll/Repeat(" in name:
        KEYS[name] = "boolean"
        print(f"\n  Auto command key: {name}")

eid_map = {}
for eid, (name, typ) in entries.items():
    if name in KEYS:
        eid_map[eid] = name

data = defaultdict(list)

def decode_swerve_states(raw_bytes):
    n = len(raw_bytes) // 16
    states = []
    for i in range(n):
        speed, angle = st.unpack_from('<dd', raw_bytes, i * 16)
        states.append((speed, angle))
    return states

def decode_chassis_speeds(raw_bytes):
    if len(raw_bytes) >= 24:
        vx, vy, omega = st.unpack_from('<ddd', raw_bytes, 0)
        return (vx, vy, omega)
    return None

for rec in reader:
    if rec.isStart() or rec.isFinish() or rec.isControl() or rec.isSetMetadata():
        continue
    eid = rec.getEntry()
    if eid not in eid_map:
        continue
    name = eid_map[eid]
    ts = rec.getTimestamp() / 1e6

    try:
        if "SwerveModuleState" in KEYS[name]:
            raw = bytes(rec.getRaw())
            if len(raw) < 16:
                continue
            states = decode_swerve_states(raw)
            data[name].append((ts, states))
        elif "ChassisSpeeds" in KEYS[name]:
            raw = bytes(rec.getRaw())
            if len(raw) < 24:
                continue
            cs = decode_chassis_speeds(raw)
            if cs:
                data[name].append((ts, cs))
        elif KEYS[name] == "boolean":
            data[name].append((ts, rec.getBoolean()))
        elif KEYS[name] == "double":
            data[name].append((ts, rec.getDouble()))
        elif KEYS[name] == "float":
            data[name].append((ts, rec.getFloat()))
    except:
        pass

print(f"\nCollected {sum(len(v) for v in data.values())} records across {len(data)} keys")

# Find auto window from command key
auto_cmd_key = None
for k in data:
    if "Repeat(" in k:
        auto_cmd_key = k
        break

auto_start = auto_end = None
teleop_start = None  # approx: auto_end + ~15s

if auto_cmd_key:
    for ts, val in data[auto_cmd_key]:
        if val and auto_start is None:
            auto_start = ts
        if auto_start and not val and auto_end is None:
            auto_end = ts
    if auto_start and auto_end:
        teleop_start = auto_end + 15  # rough estimate
        print(f"Auto: {auto_start:.2f}s - {auto_end:.2f}s ({auto_end-auto_start:.1f}s)")
    else:
        print(f"WARNING: Could not find auto window boundaries (start={auto_start}, end={auto_end})")
else:
    print("WARNING: No Repeat() auto command found in log")

# If no repeat command, try to find auto from timing (first enabled period)
if auto_start is None:
    print("Attempting to find auto from swerve setpoint activity...")
    cs_setp = data.get("/RealOutputs/SwerveChassisSpeeds/Setpoints", [])
    for ts, (vx, vy, om) in cs_setp:
        speed = math.hypot(vx, vy)
        if speed > 0.1 and auto_start is None:
            auto_start = ts
            break
    if auto_start:
        # Assume auto is ~15s long
        auto_end = auto_start + 15
        teleop_start = auto_end + 15
        print(f"Estimated auto: {auto_start:.2f}s - {auto_end:.2f}s")

def in_window(ts, start, end):
    return start <= ts <= end

setp_key = "/RealOutputs/SwerveStates/Setpoints"
meas_key = "/RealOutputs/SwerveStates/Measured"
cs_setp_key = "/RealOutputs/SwerveChassisSpeeds/Setpoints"
cs_meas_key = "/RealOutputs/SwerveChassisSpeeds/Measured"
mod_names = ["FL", "FR", "BL", "BR"]

# ============================================================
# AUTO ANALYSIS
# ============================================================
if auto_start and auto_end:
    print("\n" + "=" * 70)
    print(f"AUTO PERIOD ({auto_start:.1f}s - {auto_end:.1f}s)")
    print("=" * 70)

    setp = [(ts, s) for ts, s in data.get(setp_key, []) if in_window(ts, auto_start, auto_end)]
    meas = [(ts, s) for ts, s in data.get(meas_key, []) if in_window(ts, auto_start, auto_end)]

    print(f"\n--- Module drive velocity tracking error ---")
    if setp and meas:
        n = min(len(setp), len(meas))
        for mod in range(min(4, len(setp[0][1]) if setp else 0)):
            errors = []
            for i in range(n):
                sv = setp[i][1][mod][0]
                mv = meas[i][1][mod][0]
                errors.append(abs(sv - mv))
            avg_e = sum(errors) / len(errors)
            max_e = max(errors)
            max_i = errors.index(max_e)
            big = sum(1 for e in errors if e > 0.5)
            print(f"  {mod_names[mod]}: avg_err={avg_e:.3f} m/s, max_err={max_e:.3f} m/s @ t={setp[max_i][0]:.2f}s, big(>0.5m/s)={big}/{n}")

    # Chassis speed timeline
    cs_setp = [(ts, v) for ts, v in data.get(cs_setp_key, []) if in_window(ts, auto_start, auto_end)]
    cs_meas = [(ts, v) for ts, v in data.get(cs_meas_key, []) if in_window(ts, auto_start, auto_end)]

    print(f"\n--- Chassis speed setpoint vs measured (every 0.5s) ---")
    if cs_setp and cs_meas:
        t = auto_start
        while t <= auto_end:
            cs = min(cs_setp, key=lambda x: abs(x[0]-t))
            cm = min(cs_meas, key=lambda x: abs(x[0]-t))
            if abs(cs[0]-t) < 0.2:
                sx,sy,so = cs[1]
                mx,my,mo = cm[1]
                speed_s = math.hypot(sx,sy)
                speed_m = math.hypot(mx,my)
                err = speed_s - speed_m
                print(f"  t={t:.1f}s  setp={speed_s:.2f}m/s  meas={speed_m:.2f}m/s  err={err:+.2f}  ω_s={so:+.2f} ω_m={mo:+.2f}")
            t += 0.5

    # Setpoint velocity discontinuities
    print(f"\n--- FL setpoint velocity jumps >0.5 m/s in one cycle ---")
    if setp:
        jumps = []
        for i in range(1, len(setp)):
            dt = setp[i][0] - setp[i-1][0]
            sv_now = setp[i][1][0][0]
            sv_prev = setp[i-1][1][0][0]
            dv = abs(sv_now - sv_prev)
            if dv > 0.5 and dt < 0.05:
                jumps.append((setp[i][0], sv_prev, sv_now, dv))
        print(f"  Count: {len(jumps)}")
        for t, vb, va, dv in jumps[:10]:
            print(f"    t={t:.3f}s: {vb:+.2f} -> {va:+.2f} m/s (Δ={dv:.2f})")

    # Zero velocity periods
    if setp:
        zero_start_t = None
        zeros = []
        for ts, states in setp:
            total_speed = sum(abs(s[0]) for s in states)
            if total_speed < 0.1:
                if zero_start_t is None:
                    zero_start_t = ts
            else:
                if zero_start_t is not None:
                    zeros.append((zero_start_t, ts, ts - zero_start_t))
                    zero_start_t = None
        if zero_start_t:
            zeros.append((zero_start_t, setp[-1][0], setp[-1][0] - zero_start_t))
        print(f"\n--- Zero-velocity setpoint periods ---")
        for s, e, d in zeros:
            print(f"    {s:.2f}s - {e:.2f}s ({d:.2f}s)")

    # Voltage drops
    print(f"\n--- Applied voltage drops to 0V (all modules synchronized) ---")
    for mod in range(4):
        vkey = f"/Drive/Module{mod}/DriveAppliedVolts"
        vdata = [(ts, v) for ts, v in data.get(vkey, []) if in_window(ts, auto_start, auto_end)]
        if vdata:
            zero_drops = [(ts, v) for i, (ts, v) in enumerate(vdata) if i > 0 and abs(v) < 0.1 and abs(vdata[i-1][1]) > 2.0]
            print(f"  {mod_names[mod]}: {len(zero_drops)} drops to 0V from >2V")

    # Loop timing during auto
    timing = [(ts, v) for ts, v in data.get("/RealOutputs/LoggedRobot/FullCycleMS", []) if in_window(ts, auto_start, auto_end)]
    if timing:
        vals = [v for _, v in timing]
        avg = sum(vals)/len(vals)
        p95 = sorted(vals)[int(len(vals)*0.95)]
        mx = max(vals)
        over20 = sum(1 for v in vals if v > 20)
        over40 = sum(1 for v in vals if v > 40)
        print(f"\n--- Loop timing ---")
        print(f"  avg={avg:.1f}ms p95={p95:.1f}ms max={mx:.1f}ms >20ms={over20}/{len(vals)} >40ms={over40}/{len(vals)}")
        worst = sorted(timing, key=lambda x: -x[1])[:3]
        print(f"  Worst: {', '.join(f't={t:.2f}s:{v:.0f}ms' for t,v in worst)}")

# ============================================================
# TELEOP ANALYSIS (sample a 30s window)
# ============================================================
print("\n" + "=" * 70)
print("TELEOP PERIOD ANALYSIS")
print("=" * 70)

# Find teleop by looking for activity after auto ends
if auto_end:
    # Sample multiple windows to catch the CAN loss period
    cs_setp_all = data.get(cs_setp_key, [])
    cs_meas_all = data.get(cs_meas_key, [])
    timing_all = data.get("/RealOutputs/LoggedRobot/FullCycleMS", [])
    
    # Find actual teleop start: first setpoint after auto_end + 10s gap
    teleop_start = None
    for ts, v in cs_setp_all:
        if ts > auto_end + 10:
            teleop_start = ts
            break
    
    if teleop_start is None:
        teleop_start = auto_end + 20
    
    # Find match end
    match_end = cs_setp_all[-1][0] if cs_setp_all else teleop_start + 135
    
    print(f"Teleop window: ~{teleop_start:.1f}s - {match_end:.1f}s")
    
    # Overall teleop tracking error
    teleop_cs_setp = [(ts, v) for ts, v in cs_setp_all if in_window(ts, teleop_start, match_end)]
    teleop_cs_meas = [(ts, v) for ts, v in cs_meas_all if in_window(ts, teleop_start, match_end)]
    
    if teleop_cs_setp and teleop_cs_meas:
        n = min(len(teleop_cs_setp), len(teleop_cs_meas))
        speed_errs = []
        for i in range(n):
            sx, sy, so = teleop_cs_setp[i][1]
            mx, my, mo = teleop_cs_meas[i][1]
            speed_s = math.hypot(sx, sy)
            speed_m = math.hypot(mx, my)
            speed_errs.append(abs(speed_s - speed_m))
        avg_e = sum(speed_errs) / len(speed_errs)
        max_e = max(speed_errs)
        big = sum(1 for e in speed_errs if e > 1.0)
        print(f"\n--- Chassis speed tracking error ---")
        print(f"  avg_err={avg_e:.3f} m/s, max_err={max_e:.3f} m/s, big(>1.0m/s)={big}/{n}")
    
    # Loop timing during teleop
    teleop_timing = [(ts, v) for ts, v in timing_all if in_window(ts, teleop_start, match_end)]
    if teleop_timing:
        vals = [v for _, v in teleop_timing]
        avg = sum(vals)/len(vals)
        p95 = sorted(vals)[int(len(vals)*0.95)]
        mx = max(vals)
        over20 = sum(1 for v in vals if v > 20)
        over40 = sum(1 for v in vals if v > 40)
        print(f"\n--- Loop timing ---")
        print(f"  avg={avg:.1f}ms p95={p95:.1f}ms max={mx:.1f}ms >20ms={over20}/{len(vals)} >40ms={over40}/{len(vals)}")
        worst = sorted(teleop_timing, key=lambda x: -x[1])[:5]
        print(f"  Worst: {', '.join(f't={t:.2f}s:{v:.0f}ms' for t,v in worst)}")
        
        # Find clusters of bad timing
        spikes = [(t, v) for t, v in teleop_timing if v > 30]
        if spikes:
            print(f"\n--- Timing spike clusters (>30ms) ---")
            clusters = []
            cluster = [spikes[0]]
            for i in range(1, len(spikes)):
                if spikes[i][0] - cluster[-1][0] < 2.0:
                    cluster.append(spikes[i])
                else:
                    clusters.append(cluster)
                    cluster = [spikes[i]]
            clusters.append(cluster)
            for c in clusters[:10]:
                t_start = c[0][0]
                t_end = c[-1][0]
                max_ms = max(v for _, v in c)
                print(f"    {t_start:.1f}s-{t_end:.1f}s ({len(c)} spikes, max={max_ms:.0f}ms)")
    
    # Voltage drops during teleop
    print(f"\n--- Applied voltage drops during teleop ---")
    for mod in range(4):
        vkey = f"/Drive/Module{mod}/DriveAppliedVolts"
        vdata = [(ts, v) for ts, v in data.get(vkey, []) if in_window(ts, teleop_start, match_end)]
        if vdata:
            zero_drops = [(ts, v) for i, (ts, v) in enumerate(vdata) if i > 0 and abs(v) < 0.1 and abs(vdata[i-1][1]) > 2.0]
            print(f"  {mod_names[mod]}: {len(zero_drops)} drops to 0V from >2V")
    
    # CAN utilization during teleop
    can_util = [(ts, v) for ts, v in data.get("/SystemStats/CANBus/Utilization", []) if in_window(ts, teleop_start, match_end)]
    if can_util:
        vals = [v for _, v in can_util]
        print(f"\n--- CAN bus utilization ---")
        print(f"  avg={sum(vals)/len(vals):.1f}%, max={max(vals):.1f}%, min={min(vals):.1f}%")

print("\nDone.")
