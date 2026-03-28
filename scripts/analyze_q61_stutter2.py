"""Quick Q61 auto stutter root-cause — v2 with struct decoding."""

import struct as st
import sys, math
from collections import defaultdict
from wpiutil.log import DataLogReader

LOG = r"C:\Users\danla\Desktop\Logs Idaho\akit_26-03-28_16-55-43_idbo_q61.wpilog"

reader = DataLogReader(LOG)

# Build entry map
entries = {}
for rec in reader:
    if rec.isStart():
        d = rec.getStartData()
        entries[d.entry] = (d.name, d.type)

# Keys
KEYS = {
    "/Drive/Module0/DriveVelocityRadPerSec": "double",
    "/Drive/Module1/DriveVelocityRadPerSec": "double",
    "/Drive/Module2/DriveVelocityRadPerSec": "double",
    "/Drive/Module3/DriveVelocityRadPerSec": "double",
    "/RealOutputs/SwerveStates/Setpoints": "struct:SwerveModuleState[]",
    "/RealOutputs/SwerveStates/Measured": "struct:SwerveModuleState[]",
    "/RealOutputs/SwerveStates/SetpointsOptimized": "struct:SwerveModuleState[]",
    "/RealOutputs/SwerveChassisSpeeds/Setpoints": "struct:ChassisSpeeds",
    "/RealOutputs/SwerveChassisSpeeds/Measured": "struct:ChassisSpeeds",
    "/RealOutputs/CommandsAll/Repeat(11 - Left Curved Bum Rush)": "boolean",
    "/RealOutputs/LoggedRobot/FullCycleMS": "double",
    "/Drive/Module0/DriveAppliedVolts": "double",
    "/Drive/Module1/DriveAppliedVolts": "double",
    "/Drive/Module2/DriveAppliedVolts": "double",
    "/Drive/Module3/DriveAppliedVolts": "double",
}

eid_map = {}
for eid, (name, typ) in entries.items():
    if name in KEYS:
        eid_map[eid] = name

data = defaultdict(list)

def decode_swerve_states(raw_bytes):
    """SwerveModuleState[] = N * (double speed_mps, double angle_rad).
    AK logs Rotation2d as a single double (the angle in radians)."""
    n = len(raw_bytes) // 16  # 2 doubles per state
    states = []
    for i in range(n):
        speed, angle = st.unpack_from('<dd', raw_bytes, i * 16)
        states.append((speed, angle))
    return states

def decode_chassis_speeds(raw_bytes):
    """ChassisSpeeds = (double vx, double vy, double omega)"""
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
    except:
        pass

print(f"Collected {sum(len(v) for v in data.values())} records across {len(data)} keys")

# Find auto window from command key
cmd_key = "/RealOutputs/CommandsAll/Repeat(11 - Left Curved Bum Rush)"
auto_start = auto_end = None
for ts, val in data.get(cmd_key, []):
    if val and auto_start is None:
        auto_start = ts
    if auto_start and not val and auto_end is None:
        auto_end = ts

print(f"Auto: {auto_start:.2f}s - {auto_end:.2f}s ({auto_end-auto_start:.1f}s)\n")

def in_auto(ts):
    return auto_start <= ts <= auto_end

# ============================================================
# 1) Setpoint vs Measured velocity per module
# ============================================================
print("=" * 70)
print("1) SETPOINT vs MEASURED drive velocity per module during auto")
print("=" * 70)

setp_key = "/RealOutputs/SwerveStates/Setpoints"
meas_key = "/RealOutputs/SwerveStates/Measured"
opt_key = "/RealOutputs/SwerveStates/SetpointsOptimized"

setp = [(ts, s) for ts, s in data.get(setp_key, []) if in_auto(ts)]
meas = [(ts, s) for ts, s in data.get(meas_key, []) if in_auto(ts)]
opt = [(ts, s) for ts, s in data.get(opt_key, []) if in_auto(ts)]

mod_names = ["FL", "FR", "BL", "BR"]

if setp and meas:
    # Align by index (they should be logged at same rate)
    n = min(len(setp), len(meas))
    for mod in range(4):
        errors = []
        for i in range(n):
            sv = setp[i][1][mod][0]  # speed m/s
            mv = meas[i][1][mod][0]
            errors.append(abs(sv - mv))
        avg_e = sum(errors) / len(errors)
        max_e = max(errors)
        max_i = errors.index(max_e)
        big = sum(1 for e in errors if e > 0.5)  # >0.5 m/s tracking error
        print(f"  {mod_names[mod]}: avg_err={avg_e:.3f} m/s, max_err={max_e:.3f} m/s @ t={setp[max_i][0]:.2f}s, big(>0.5m/s)={big}/{n}")
else:
    print("  Missing setpoint or measured swerve state data")

# ============================================================
# 2) Setpoint velocity timeline — detect discontinuities
# ============================================================
print("\n" + "=" * 70)
print("2) SETPOINT velocity discontinuities (FL speed, m/s)")
print("=" * 70)

if setp:
    jumps = []
    for i in range(1, len(setp)):
        dt = setp[i][0] - setp[i-1][0]
        sv_now = setp[i][1][0][0]
        sv_prev = setp[i-1][1][0][0]
        dv = abs(sv_now - sv_prev)
        if dv > 0.5 and dt < 0.05:
            jumps.append((setp[i][0], sv_prev, sv_now, dv))

    print(f"  Jumps >0.5 m/s in one cycle: {len(jumps)}")
    for t, vb, va, dv in jumps[:15]:
        print(f"    t={t:.3f}s: {vb:+.2f} -> {va:+.2f}  (Δ={dv:.2f} m/s)")

    # Zero-velocity periods
    zero_start = None
    zeros = []
    for ts, states in setp:
        total_speed = sum(abs(s[0]) for s in states)
        if total_speed < 0.1:
            if zero_start is None:
                zero_start = ts
        else:
            if zero_start is not None:
                zeros.append((zero_start, ts, ts - zero_start))
                zero_start = None
    if zero_start:
        zeros.append((zero_start, setp[-1][0], setp[-1][0] - zero_start))
    print(f"\n  Zero-velocity periods (all modules <0.1 m/s total):")
    for s, e, d in zeros:
        print(f"    {s:.2f}s - {e:.2f}s ({d:.2f}s)")

# ============================================================
# 3) Turn angle setpoint discontinuities (wheel fight detection)
# ============================================================
print("\n" + "=" * 70)
print("3) TURN ANGLE setpoint jumps during auto (wheel fight?)")
print("=" * 70)

if opt:  # use optimized setpoints — these are what actually goes to motors
    for mod in range(4):
        angle_jumps = []
        for i in range(1, len(opt)):
            dt = opt[i][0] - opt[i-1][0]
            a_now = opt[i][1][mod][1]
            a_prev = opt[i-1][1][mod][1]
            da = (a_now - a_prev + math.pi) % (2*math.pi) - math.pi
            if abs(da) > 0.3 and dt < 0.05:  # >17 degrees in one cycle
                angle_jumps.append((opt[i][0], math.degrees(a_prev), math.degrees(a_now), math.degrees(da)))
        print(f"  {mod_names[mod]}: angle jumps >17° in one cycle: {len(angle_jumps)}")
        for t, ab, aa, da in angle_jumps[:5]:
            print(f"    t={t:.3f}s: {ab:.1f}° -> {aa:.1f}° (Δ={da:+.1f}°)")

# ============================================================
# 4) Chassis-level speeds: setpoint vs measured
# ============================================================
print("\n" + "=" * 70)
print("4) CHASSIS SPEEDS setpoint vs measured during auto")
print("=" * 70)

cs_setp = [(ts, v) for ts, v in data.get("/RealOutputs/SwerveChassisSpeeds/Setpoints", []) if in_auto(ts)]
cs_meas = [(ts, v) for ts, v in data.get("/RealOutputs/SwerveChassisSpeeds/Measured", []) if in_auto(ts)]

if cs_setp and cs_meas:
    n = min(len(cs_setp), len(cs_meas))
    vx_errs, vy_errs, om_errs = [], [], []
    for i in range(n):
        sx, sy, so = cs_setp[i][1]
        mx, my, mo = cs_meas[i][1]
        vx_errs.append(abs(sx - mx))
        vy_errs.append(abs(sy - my))
        om_errs.append(abs(so - mo))
    print(f"  vx: avg_err={sum(vx_errs)/n:.3f} m/s, max={max(vx_errs):.3f}")
    print(f"  vy: avg_err={sum(vy_errs)/n:.3f} m/s, max={max(vy_errs):.3f}")
    print(f"  ω:  avg_err={sum(om_errs)/n:.3f} rad/s, max={max(om_errs):.3f}")

    # Sample timeline
    print("\n  Timeline (every 0.5s):")
    t = auto_start
    while t <= auto_end:
        cs = min(cs_setp, key=lambda x: abs(x[0]-t))
        cm = min(cs_meas, key=lambda x: abs(x[0]-t))
        if abs(cs[0]-t) < 0.1:
            sx,sy,so = cs[1]
            mx,my,mo = cm[1]
            speed_s = math.hypot(sx,sy)
            speed_m = math.hypot(mx,my)
            print(f"    t={t:.1f}s  setp=({sx:+.2f},{sy:+.2f})={speed_s:.2f}m/s ω={so:+.2f}  meas=({mx:+.2f},{my:+.2f})={speed_m:.2f}m/s ω={mo:+.2f}")
        t += 0.5

# ============================================================
# 5) Loop timing
# ============================================================
print("\n" + "=" * 70)
print("5) LOOP TIMING during auto")
print("=" * 70)

timing = [(ts, v) for ts, v in data.get("/RealOutputs/LoggedRobot/FullCycleMS", []) if in_auto(ts)]
if timing:
    vals = [v for _, v in timing]
    avg = sum(vals)/len(vals)
    p95 = sorted(vals)[int(len(vals)*0.95)]
    mx = max(vals)
    over20 = sum(1 for v in vals if v > 20)
    over40 = sum(1 for v in vals if v > 40)
    print(f"  avg={avg:.1f}ms p95={p95:.1f}ms max={mx:.1f}ms >20ms={over20}/{len(vals)} >40ms={over40}/{len(vals)}")
    worst = sorted(timing, key=lambda x: -x[1])[:5]
    print(f"  Worst spikes: {', '.join(f't={t:.2f}s:{v:.0f}ms' for t,v in worst)}")

# ============================================================
# 6) Applied voltage discontinuities
# ============================================================
print("\n" + "=" * 70)
print("6) APPLIED VOLTAGE discontinuities during auto")
print("=" * 70)

for mod in range(4):
    vkey = f"/Drive/Module{mod}/DriveAppliedVolts"
    vdata = [(ts, v) for ts, v in data.get(vkey, []) if in_auto(ts)]
    if not vdata:
        print(f"  {mod_names[mod]}: No data")
        continue
    jumps = []
    for i in range(1, len(vdata)):
        dt = vdata[i][0] - vdata[i-1][0]
        dv = abs(vdata[i][1] - vdata[i-1][1])
        if dv > 3.0 and dt < 0.05:
            jumps.append((vdata[i][0], vdata[i-1][1], vdata[i][1], dv))
    print(f"  {mod_names[mod]}: voltage jumps >3V: {len(jumps)}")
    for t, vb, va, dv in jumps[:5]:
        print(f"    t={t:.3f}s: {vb:+.1f}V -> {va:+.1f}V (Δ={dv:.1f}V)")

# ============================================================
# 7) Correlate timing spikes with velocity jumps
# ============================================================
print("\n" + "=" * 70)
print("7) CORRELATION: timing spikes vs velocity setpoint changes")
print("=" * 70)

if timing and setp:
    spike_times = [t for t, v in timing if v > 30]
    print(f"  Loop spikes >30ms: {len(spike_times)}")
    for spike_t in spike_times[:10]:
        # Find setpoint change near this spike
        closest_setp_i = min(range(len(setp)), key=lambda i: abs(setp[i][0] - spike_t))
        if closest_setp_i > 0:
            sv_before = setp[closest_setp_i-1][1][0][0]
            sv_after = setp[closest_setp_i][1][0][0]
            dv = abs(sv_after - sv_before)
            cycle_ms = min(timing, key=lambda x: abs(x[0]-spike_t))[1]
            print(f"    spike t={spike_t:.3f}s ({cycle_ms:.0f}ms): FL setp {sv_before:+.2f} -> {sv_after:+.2f} m/s (Δ={dv:.2f})")

print("\nDone.")
