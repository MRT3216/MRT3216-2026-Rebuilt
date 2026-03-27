"""
MRT3216 — Idaho Log Analyzer v2
Properly handles sparse boolean signals by interpolating state over time windows.

Usage:
    .venv/Scripts/python.exe scripts/analyze_idaho_v2.py <path-to-wpilog>
"""

import sys, os, bisect, math
from collections import defaultdict
from wpiutil.log import DataLogReader

if len(sys.argv) < 2:
    print("Usage: python analyze_idaho_v2.py <path-to-wpilog>")
    sys.exit(1)

LOG_PATH = sys.argv[1]
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PATH = os.path.join(SCRIPT_DIR, "idaho_analysis_v2.txt")
_out = open(OUTPUT_PATH, "w", encoding="utf-8")

def p(*args, **kwargs):
    print(*args, **kwargs)
    print(*args, **kwargs, file=_out)
    _out.flush()

# ── Parse everything into typed storage ──────────────────────────────────────
entry_map = {}
type_map = {}
numeric = defaultdict(list)      # name -> [(t, val)]
booleans = defaultdict(list)     # name -> [(t, bool)]
strings = defaultdict(list)      # name -> [(t, str)]
string_arrays = defaultdict(list) # name -> [(t, [str])]
doubles_arr = defaultdict(list)  # name -> [(t, [float])]

p(f"Parsing {LOG_PATH}...")

for record in DataLogReader(LOG_PATH):
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
        if typ in ("double", "float"):
            numeric[name].append((t, record.getDouble()))
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

p(f"  Parsed {len(numeric)} numeric, {len(booleans)} boolean, {len(strings)} string, {len(string_arrays)} string[] signals")

# ── Match timeline from DriverStation ────────────────────────────────────────
en_pts = booleans["/DriverStation/Enabled"]
au_pts = booleans["/DriverStation/Autonomous"]
mt_pts = numeric["/DriverStation/MatchTime"]

# Find phases
auto_start = auto_end = teleop_start = teleop_end = None
for t, v in en_pts:
    if v and auto_start is None:
        # Check if autonomous
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

teleop_end = None
for t, v in en_pts:
    if not v and teleop_start and t > teleop_start:
        teleop_end = t

# ── Helper: get value at time by last-known state ────────────────────────────
def bool_at(name, t):
    pts = booleans.get(name, [])
    val = False
    for pt, pv in pts:
        if pt <= t: val = pv
        else: break
    return val

def numeric_in_window(name, t_start, t_end):
    pts = numeric.get(name, [])
    return [(t, v) for t, v in pts if t_start <= t <= t_end]

def bool_in_window(name, t_start, t_end):
    pts = booleans.get(name, [])
    return [(t, v) for t, v in pts if t_start <= t <= t_end]

def str_in_window(name, t_start, t_end):
    pts = strings.get(name, [])
    return [(t, v) for t, v in pts if t_start <= t <= t_end]

def strarr_in_window(name, t_start, t_end):
    pts = string_arrays.get(name, [])
    return [(t, v) for t, v in pts if t_start <= t <= t_end]

def stats(pairs):
    if not pairs:
        return None
    vals = [v for _, v in pairs]
    n = len(vals)
    avg = sum(vals) / n
    mn = min(vals)
    mx = max(vals)
    # Percentiles
    sv = sorted(vals)
    p50 = sv[n // 2]
    p95 = sv[int(n * 0.95)]
    p99 = sv[int(n * 0.99)]
    return {"n": n, "min": mn, "max": mx, "avg": avg, "p50": p50, "p95": p95, "p99": p99}

def bool_duty_cycle(name, t_start, t_end, sample_hz=50):
    """Calculate fraction of time a boolean signal is True in a window."""
    pts = booleans.get(name, [])
    if not pts:
        return None
    # Build state timeline
    state = False
    for pt, pv in pts:
        if pt <= t_start:
            state = pv
    # Sample at fixed rate
    dt = 1.0 / sample_hz
    total = 0
    active = 0
    t = t_start
    transitions = [(pt, pv) for pt, pv in pts if t_start < pt <= t_end]
    ti = 0
    while t <= t_end:
        while ti < len(transitions) and transitions[ti][0] <= t:
            state = transitions[ti][1]
            ti += 1
        total += 1
        if state:
            active += 1
        t += dt
    return active / total if total > 0 else 0

# ── Report ───────────────────────────────────────────────────────────────────
log_dur = max(t for t, _ in en_pts) - min(t for t, _ in en_pts)
p("=" * 80)
p("  MRT3216 IDAHO LOG ANALYSIS v2")
p(f"  File: {os.path.basename(LOG_PATH)}")
p(f"  Size: {os.path.getsize(LOG_PATH) / 1e6:.1f} MB")
p(f"  Total log duration: {log_dur:.1f}s")

# Match info
mn = numeric.get("/DriverStation/MatchNumber", [])
mt = numeric.get("/DriverStation/MatchType", [])
if mn: p(f"  Match #: {int(mn[-1][1])}")
if mt:
    v = int(mt[-1][1])
    p(f"  Match type: {['None','Practice','Qual','Elim'][v] if v < 4 else v}")

fms = booleans.get("/DriverStation/FMSAttached", [])
fms_on = any(v for _, v in fms)
p(f"  FMS attached: {'Yes' if fms_on else 'No'}")

alliance = numeric.get("/DriverStation/AllianceStation", [])
if alliance:
    a = int(alliance[-1][1])
    # 0=R1,1=R2,2=R3,3=B1,4=B2,5=B3
    labels = ["Red1","Red2","Red3","Blue1","Blue2","Blue3"]
    p(f"  Alliance station: {labels[a] if a < 6 else a}")

if auto_start and auto_end:
    p(f"\n  AUTO:   {auto_start:.1f}s – {auto_end:.1f}s  ({auto_end-auto_start:.1f}s)")
if teleop_start and teleop_end:
    p(f"  TELEOP: {teleop_start:.1f}s – {teleop_end:.1f}s  ({teleop_end-teleop_start:.1f}s)")
p("=" * 80)

TS = teleop_start or 0
TE = teleop_end or 0
AS = auto_start or 0
AE = auto_end or 0

# ── CRT Encoder ──────────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  CRT ENCODER STATUS")
p("─" * 80)
crt_status = strings.get("/RealOutputs/Shooter/Turret/CRT/Status", [])
if crt_status:
    for t, s in crt_status[:3]:
        p(f"  t={t:.2f}s  Status={s}")
    crt_err = numeric.get("/RealOutputs/Shooter/Turret/CRT/ErrorRot", [])
    crt_res = numeric.get("/RealOutputs/Shooter/Turret/CRT/ResolvedDeg", [])
    crt_att = numeric.get("/RealOutputs/Shooter/Turret/CRT/AttemptsUsed", [])
    if crt_err: p(f"  Error: {crt_err[0][1]:.6f} rot")
    if crt_res: p(f"  Resolved: {crt_res[0][1]:.2f} deg")
    if crt_att: p(f"  Attempts: {int(crt_att[0][1])}")
else:
    p("  (no CRT data)")

# ── Battery / Power ──────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  BATTERY / POWER (teleop)")
p("─" * 80)
bv = numeric_in_window("/SystemStats/BatteryVoltage", TS, TE)
bc = numeric_in_window("/SystemStats/BatteryCurrent", TS, TE)
bv_s = stats(bv)
bc_s = stats(bc)
if bv_s:
    p(f"  Voltage:  avg={bv_s['avg']:.2f}V  min={bv_s['min']:.2f}V  p5={sorted([v for _,v in bv])[int(len(bv)*0.05)]:.2f}V  max={bv_s['max']:.2f}V")
if bc_s:
    p(f"  Current:  avg={bc_s['avg']:.1f}A  p95={bc_s['p95']:.1f}A  max={bc_s['max']:.1f}A")

# Brownout
bo_duty = bool_duty_cycle("/SystemStats/BrownedOut", TS, TE)
if bo_duty is not None:
    if bo_duty > 0:
        p(f"  *** BROWNOUT: active {bo_duty*100:.1f}% of teleop ***")
    else:
        p("  No brownouts during teleop")

# Voltage sag zones
if bv:
    below8 = [(t, v) for t, v in bv if v < 8.0]
    below10 = [(t, v) for t, v in bv if v < 10.0]
    if below8:
        worst = min(below8, key=lambda x: x[1])
        p(f"  *** {len(below8)} samples below 8V — worst: {worst[1]:.2f}V at t={worst[0]:.1f}s ***")
    if below10:
        p(f"  *** {len(below10)} samples below 10V ***")
    # Histogram
    p("  Voltage distribution:")
    bins = [(6,7),(7,8),(8,9),(9,10),(10,11),(11,12),(12,13),(13,14)]
    for lo, hi in bins:
        cnt = sum(1 for _,v in bv if lo <= v < hi)
        bar = "█" * (cnt * 60 // len(bv)) if len(bv) > 0 else ""
        p(f"    {lo:>2}-{hi}V: {cnt:>5} {bar}")

# Energy
te_energy = numeric.get("/RealOutputs/EnergyLogger/TotalEnergyJoules", [])
if te_energy:
    e_start = [v for t,v in te_energy if t <= TS]
    e_end = [v for t,v in te_energy if t <= TE]
    if e_start and e_end:
        delta = e_end[-1] - e_start[-1]
        p(f"  Energy (teleop): {delta:.0f} J  ({delta/3600:.1f} Wh)")

# CPU temp
cpu = numeric_in_window("/SystemStats/CPUTempCelsius", TS, TE)
cpu_s = stats(cpu)
if cpu_s:
    p(f"  CPU temp: avg={cpu_s['avg']:.1f}°C  max={cpu_s['max']:.1f}°C")

# CAN
can_util = numeric_in_window("/SystemStats/CANBus/Utilization", TS, TE)
can_s = stats(can_util)
if can_s:
    p(f"  CAN util: avg={can_s['avg']*100:.1f}%  p95={can_s['p95']*100:.1f}%  max={can_s['max']*100:.1f}%")

can_off = numeric.get("/SystemStats/CANBus/OffCount", [])
if can_off:
    p(f"  CAN off count: {int(max(v for _,v in can_off))}")

# ── Loop Timing ──────────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  LOOP TIMING (enabled)")
p("─" * 80)
# Combine auto+teleop
loop_data = []
user_data = []
if AS and AE:
    loop_data.extend(numeric_in_window("/RealOutputs/LoggedRobot/FullCycleMS", AS, AE))
    user_data.extend(numeric_in_window("/RealOutputs/LoggedRobot/UserCodeMS", AS, AE))
if TS and TE:
    loop_data.extend(numeric_in_window("/RealOutputs/LoggedRobot/FullCycleMS", TS, TE))
    user_data.extend(numeric_in_window("/RealOutputs/LoggedRobot/UserCodeMS", TS, TE))

loop_s = stats(loop_data)
user_s = stats(user_data)
if loop_s:
    p(f"  Full cycle:  avg={loop_s['avg']:.2f}ms  p95={loop_s['p95']:.2f}ms  p99={loop_s['p99']:.2f}ms  max={loop_s['max']:.2f}ms")
    overruns_20 = sum(1 for _,v in loop_data if v > 20)
    overruns_25 = sum(1 for _,v in loop_data if v > 25)
    overruns_40 = sum(1 for _,v in loop_data if v > 40)
    p(f"  Overruns:  >20ms={overruns_20} ({100*overruns_20/loop_s['n']:.1f}%)  >25ms={overruns_25}  >40ms={overruns_40}")
if user_s:
    p(f"  User code:   avg={user_s['avg']:.2f}ms  p95={user_s['p95']:.2f}ms  p99={user_s['p99']:.2f}ms  max={user_s['max']:.2f}ms")

# Worst loop spikes
if loop_data:
    worst10 = sorted(loop_data, key=lambda x: -x[1])[:10]
    p("  10 worst loop cycles:")
    for t, v in worst10:
        phase = "auto" if AS <= t <= AE else "tele" if TS <= t <= TE else "?"
        p(f"    t={t:.1f}s ({phase})  {v:.1f}ms")

queued = numeric_in_window("/RealOutputs/Logger/QueuedCycles", TS, TE)
q_s = stats(queued)
if q_s:
    p(f"  Logger queue: avg={q_s['avg']:.1f}  max={int(q_s['max'])}")

# ── Drive ────────────────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  DRIVE (teleop)")
p("─" * 80)
for i in range(4):
    dc = numeric_in_window(f"/Drive/Module{i}/DriveCurrentAmps", TS, TE)
    tc = numeric_in_window(f"/Drive/Module{i}/TurnCurrentAmps", TS, TE)
    dc_s = stats(dc)
    tc_s = stats(tc)
    if dc_s:
        p(f"  Mod{i} Drive: avg={dc_s['avg']:.1f}A  p95={dc_s['p95']:.1f}A  max={dc_s['max']:.1f}A")
    if tc_s:
        p(f"  Mod{i} Turn:  avg={tc_s['avg']:.1f}A  p95={tc_s['p95']:.1f}A  max={tc_s['max']:.1f}A")

# ── Shooter Pipeline ─────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  SHOOTER PIPELINE (teleop)")
p("─" * 80)

# Flywheel
fw_vel = numeric_in_window("/RealOutputs/Flywheel/FX/VelocityRPM", TS, TE)
fw_ref = numeric_in_window("/RealOutputs/Flywheel/FX/ReferenceRPM", TS, TE)
fw_cur = numeric_in_window("/Shooter/Flywheel/Current", TS, TE)

p("  ── Flywheel ──")
if fw_vel:
    # Filter to when reference > 100 (actively commanded)
    active_vel = [(t, v) for t, v in fw_vel]
    active_ref = [(t, v) for t, v in fw_ref]
    # Build index for cross-reference
    ref_by_time = {round(t, 3): v for t, v in fw_ref}
    
    commanded = [(t, v) for t, v in fw_ref if abs(v) > 100]
    idle = [(t, v) for t, v in fw_ref if abs(v) <= 100]
    p(f"  Total samples: {len(fw_vel)}  Commanded: {len(commanded)}  Idle: {len(idle)}")
    
    if commanded:
        cmd_s = stats(commanded)
        p(f"  Reference (when active): avg={cmd_s['avg']:.0f}  min={cmd_s['min']:.0f}  max={cmd_s['max']:.0f} RPM")
    
    # RPM tracking error
    if len(fw_vel) == len(fw_ref):
        errors = []
        for (t1, vel), (t2, ref) in zip(fw_vel, fw_ref):
            if abs(ref) > 100:
                errors.append((t1, vel - ref))
        if errors:
            err_abs = [(t, abs(e)) for t, e in errors]
            err_s = stats(err_abs)
            p(f"  Tracking error (when active): avg={err_s['avg']:.0f}  p95={err_s['p95']:.0f}  max={err_s['max']:.0f} RPM")
            # Spinup time: how long from reference change to within 50 RPM?
            big_errors = [(t, e) for t, e in err_abs if e > 200]
            if big_errors:
                p(f"  Samples with >200 RPM error: {len(big_errors)} ({100*len(big_errors)/len(errors):.1f}%)")

if fw_cur:
    fc_s = stats(fw_cur)
    p(f"  Current: avg={fc_s['avg']:.1f}A  p95={fc_s['p95']:.1f}A  max={fc_s['max']:.1f}A")

# IsSpunUp duty cycle
spun_duty = bool_duty_cycle("/RealOutputs/Flywheel/IsSpunUp", TS, TE)
if spun_duty is not None:
    p(f"  Spun-up duty cycle: {spun_duty*100:.1f}% of teleop")

# Hood
p("\n  ── Hood ──")
hood_pos = numeric_in_window("/RealOutputs/Hood/FX/PositionDegrees", TS, TE)
hood_ref = numeric_in_window("/RealOutputs/Hood/FX/ReferenceDegrees", TS, TE)
hood_cur = numeric_in_window("/Hood/Current", TS, TE)
if hood_pos:
    hp_s = stats(hood_pos)
    p(f"  Position: avg={hp_s['avg']:.1f}°  min={hp_s['min']:.1f}°  max={hp_s['max']:.1f}°")
if hood_ref:
    hr_s = stats(hood_ref)
    p(f"  Reference: avg={hr_s['avg']:.1f}°  min={hr_s['min']:.1f}°  max={hr_s['max']:.1f}°")
if hood_pos and hood_ref and len(hood_pos) == len(hood_ref):
    hood_err = [(t, abs(p - r)) for (t, p), (_, r) in zip(hood_pos, hood_ref)]
    he_s = stats(hood_err)
    p(f"  Tracking error: avg={he_s['avg']:.2f}°  p95={he_s['p95']:.2f}°  max={he_s['max']:.2f}°")
if hood_cur:
    hc_s = stats(hood_cur)
    p(f"  Current: avg={hc_s['avg']:.1f}A  p95={hc_s['p95']:.1f}A  max={hc_s['max']:.1f}A")

# Kicker
p("\n  ── Kicker ──")
k_cur = numeric_in_window("/Kicker/Current", TS, TE)
k_vel = numeric_in_window("/Kicker/Velocity", TS, TE)
k_sp = numeric_in_window("/Kicker/Setpoint", TS, TE)
if k_cur:
    kc_s = stats(k_cur)
    p(f"  Current: avg={kc_s['avg']:.1f}A  p95={kc_s['p95']:.1f}A  max={kc_s['max']:.1f}A")
if k_vel:
    active_k = [(t, v) for t, v in k_vel if abs(v) > 0.5]
    p(f"  Active time: ~{len(active_k)*0.02:.1f}s / {len(k_vel)*0.02:.1f}s")

# Spindexer
p("\n  ── Spindexer ──")
sp_cur = numeric_in_window("/Spindexer/Current", TS, TE)
sp_vel = numeric_in_window("/Spindexer/MechanismVelocity", TS, TE)
if sp_cur:
    sc_s = stats(sp_cur)
    p(f"  Current: avg={sc_s['avg']:.1f}A  p95={sc_s['p95']:.1f}A  max={sc_s['max']:.1f}A")
if sp_vel:
    active_sp = [(t, v) for t, v in sp_vel if abs(v) > 0.5]
    p(f"  Active time: ~{len(active_sp)*0.02:.1f}s / {len(sp_vel)*0.02:.1f}s")

# ── Turret ───────────────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  TURRET (teleop)")
p("─" * 80)
turret_pos = numeric_in_window("/RealOutputs/Shooter/Turret/PositionDegrees", TS, TE)
turret_cur = numeric_in_window("/Shooter/Turret/Current", TS, TE)
turret_sp = numeric_in_window("/Shooter/Turret/Setpoint", TS, TE)
if turret_pos:
    tp_s = stats(turret_pos)
    p(f"  Position: avg={tp_s['avg']:.1f}°  min={tp_s['min']:.1f}°  max={tp_s['max']:.1f}°")
if turret_cur:
    tc_s = stats(turret_cur)
    p(f"  Current: avg={tc_s['avg']:.1f}A  p95={tc_s['p95']:.1f}A  max={tc_s['max']:.1f}A")

# Turret travel — show if it's hitting the mechanical limits
if turret_pos:
    near_min = sum(1 for _,v in turret_pos if v < -85)
    near_max = sum(1 for _,v in turret_pos if v > 125)
    if near_min or near_max:
        p(f"  Near limits: {near_min} samples near -90° min, {near_max} samples near +130° max")

# ── Hybrid Aiming ────────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  HYBRID AIMING (teleop)")
p("─" * 80)
aim_duty = bool_duty_cycle("/RealOutputs/HybridAiming/aimEnabled", TS, TE)
if aim_duty is not None:
    p(f"  Aim enabled: {aim_duty*100:.1f}% of teleop")

raw_az = numeric_in_window("/RealOutputs/HybridAiming/rawTurretAzimuthDeg", TS, TE)
clamp_az = numeric_in_window("/RealOutputs/HybridAiming/clampedTurretAzimuthDeg", TS, TE)
if raw_az:
    ra_s = stats(raw_az)
    p(f"  Raw azimuth: min={ra_s['min']:.1f}°  avg={ra_s['avg']:.1f}°  max={ra_s['max']:.1f}°")
if clamp_az:
    ca_s = stats(clamp_az)
    p(f"  Clamped azimuth: min={ca_s['min']:.1f}°  avg={ca_s['avg']:.1f}°  max={ca_s['max']:.1f}°")

clamped_duty = bool_duty_cycle("/RealOutputs/HybridAiming/turretClamped", TS, TE)
outside_duty = bool_duty_cycle("/RealOutputs/HybridAiming/outsideTravelWindow", TS, TE)
if clamped_duty is not None:
    p(f"  Turret clamped: {clamped_duty*100:.1f}% of teleop")
if outside_duty is not None:
    p(f"  Outside travel window: {outside_duty*100:.1f}% of teleop")

ramp = numeric_in_window("/RealOutputs/HybridAiming/rampFactor", TS, TE)
if ramp:
    rp_s = stats(ramp)
    p(f"  Ramp factor: avg={rp_s['avg']:.2f}  max={rp_s['max']:.2f}")

# ── Hub Shift ────────────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  HUB SHIFT (teleop)")
p("─" * 80)
shifted_duty = bool_duty_cycle("/RealOutputs/HubShift/ShiftedActive", TS, TE)
active_duty = bool_duty_cycle("/RealOutputs/HubShift/Active", TS, TE)
if active_duty is not None:
    p(f"  Active (unshifted): {active_duty*100:.1f}% of teleop")
if shifted_duty is not None:
    p(f"  Shifted-active: {shifted_duty*100:.1f}% of teleop")

shift_strs = str_in_window("/RealOutputs/HubShift/CurrentShift", TS, TE)
if shift_strs:
    prev = None
    p("  Shift transitions:")
    for t, s in shift_strs:
        if s != prev:
            elapsed = t - TS
            p(f"    T+{elapsed:.0f}s (t={t:.1f}s) → {s}")
            prev = s

remain = numeric_in_window("/RealOutputs/HubShift/RemainingTime", TS, TE)
if remain:
    p(f"  Remaining time range: {min(v for _,v in remain):.1f}s – {max(v for _,v in remain):.1f}s")

# ── Shooter Telemetry ────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  SHOOTER TELEMETRY (teleop)")
p("─" * 80)
hub_dist = numeric_in_window("/RealOutputs/ShooterTelemetry/hubDistanceMeters", TS, TE)
model_rpm = numeric_in_window("/RealOutputs/ShooterTelemetry/modelRPM", TS, TE)
fudged_rpm = numeric_in_window("/RealOutputs/ShooterTelemetry/fudgedRPM", TS, TE)
rpm_fudge = numeric_in_window("/RealOutputs/ShooterTelemetry/rpmFudgeRPM", TS, TE)
lut_hood = numeric_in_window("/RealOutputs/ShooterTelemetry/lutHoodDegrees", TS, TE)
lut_tof = numeric_in_window("/RealOutputs/ShooterTelemetry/lutToFSeconds", TS, TE)

valid_duty = bool_duty_cycle("/RealOutputs/ShooterTelemetry/isValid", TS, TE)
if valid_duty is not None:
    p(f"  Solution valid: {valid_duty*100:.1f}% of teleop")

if hub_dist:
    # Filter to non-zero (when actively tracking)
    active_dist = [(t, v) for t, v in hub_dist if v > 0.1]
    if active_dist:
        hd_s = stats(active_dist)
        p(f"  Hub distance: avg={hd_s['avg']:.2f}m  min={hd_s['min']:.2f}m  max={hd_s['max']:.2f}m")

if model_rpm:
    active_mr = [(t, v) for t, v in model_rpm if v > 100]
    if active_mr:
        mr_s = stats(active_mr)
        p(f"  Model RPM: avg={mr_s['avg']:.0f}  min={mr_s['min']:.0f}  max={mr_s['max']:.0f}")

if fudged_rpm:
    active_fr = [(t, v) for t, v in fudged_rpm if v > 100]
    if active_fr:
        fr_s = stats(active_fr)
        p(f"  Fudged RPM: avg={fr_s['avg']:.0f}  min={fr_s['min']:.0f}  max={fr_s['max']:.0f}")

if rpm_fudge:
    rf_s = stats(rpm_fudge)
    p(f"  RPM fudge offset: latest={rpm_fudge[-1][1]:.0f}  min={rf_s['min']:.0f}  max={rf_s['max']:.0f}")

if lut_hood:
    active_lh = [(t, v) for t, v in lut_hood if not math.isnan(v)]
    if active_lh:
        lh_s = stats(active_lh)
        p(f"  LUT hood angle: avg={lh_s['avg']:.1f}°  min={lh_s['min']:.1f}°  max={lh_s['max']:.1f}°")

if lut_tof:
    active_tof = [(t, v) for t, v in lut_tof if v > 0]
    if active_tof:
        tof_s = stats(active_tof)
        p(f"  LUT time-of-flight: avg={tof_s['avg']:.3f}s  min={tof_s['min']:.3f}s  max={tof_s['max']:.3f}s")

shoot_mode = str_in_window("/RealOutputs/ShooterTelemetry/shootMode", TS, TE)
if shoot_mode:
    prev = None
    p("  Shoot mode changes:")
    for t, s in shoot_mode:
        if s != prev:
            p(f"    T+{t-TS:.0f}s → {s}")
            prev = s

# ── Intake ───────────────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  INTAKE (teleop)")
p("─" * 80)
piv_vel = numeric_in_window("/Intake/Pivot/Velocity", TS, TE)
roll_vel = numeric_in_window("/Intake/Rollers/Velocity", TS, TE)
if piv_vel:
    pv_s = stats(piv_vel)
    p(f"  Pivot velocity: avg={pv_s['avg']:.1f}  max={pv_s['max']:.1f}")
if roll_vel:
    active_r = [(t, v) for t, v in roll_vel if abs(v) > 0.5]
    p(f"  Rollers active time: ~{len(active_r)*0.02:.1f}s / {len(roll_vel)*0.02:.1f}s")

# ── Commands Timeline ────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  COMMAND ACTIVITY (teleop)")
p("─" * 80)
# Find all command signals
cmd_prefix = "/RealOutputs/CommandsAll/"
cmd_names = [n for n in booleans if n.startswith(cmd_prefix)]
cmd_duties = []
for name in sorted(cmd_names):
    duty = bool_duty_cycle(name, TS, TE)
    if duty is not None and duty > 0.001:
        label = name[len(cmd_prefix):]
        active_s = duty * (TE - TS)
        cmd_duties.append((duty, active_s, label))

cmd_duties.sort(key=lambda x: -x[0])
for duty, active_s, label in cmd_duties:
    p(f"  {label:<45} {active_s:>5.1f}s  ({duty*100:>5.1f}%)")

if not cmd_duties:
    p("  (no active commands found)")

# ── Vision ───────────────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  VISION (teleop)")
p("─" * 80)
# Check per-camera data
for cam in range(4):
    total = doubles_arr.get(f"/RealOutputs/Vision/Camera{cam}/RobotPoses", [])
    accepted = doubles_arr.get(f"/RealOutputs/Vision/Camera{cam}/RobotPosesAccepted", [])
    rejected = doubles_arr.get(f"/RealOutputs/Vision/Camera{cam}/RobotPosesRejected", [])
    
    total_tele = [(t, v) for t, v in total if TS <= t <= TE]
    acc_tele = [(t, v) for t, v in accepted if TS <= t <= TE]
    rej_tele = [(t, v) for t, v in rejected if TS <= t <= TE]
    
    # Count non-empty arrays
    total_nonempty = sum(1 for _, v in total_tele if len(v) > 0)
    acc_nonempty = sum(1 for _, v in acc_tele if len(v) > 0)
    rej_nonempty = sum(1 for _, v in rej_tele if len(v) > 0)
    
    if total_tele:
        p(f"  Camera {cam}: {len(total_tele)} frames, {total_nonempty} with poses, {acc_nonempty} accepted, {rej_nonempty} rejected")
    else:
        p(f"  Camera {cam}: no data")

# Alerts about cameras
alerts_warn = string_arrays.get("/RealOutputs/Alerts/warnings", [])
cam_alerts = set()
for t, arr in alerts_warn:
    for w in arr:
        if "camera" in w.lower() or "vision" in w.lower():
            cam_alerts.add(w)
for w in sorted(cam_alerts):
    p(f"  ALERT: {w}")

# ── Alerts ───────────────────────────────────────────────────────────────────
p("\n" + "─" * 80)
p("  ALERTS")
p("─" * 80)
all_errors = set()
all_warnings = set()
for t, arr in string_arrays.get("/RealOutputs/Alerts/errors", []):
    for e in arr:
        if e: all_errors.add(e)
for t, arr in string_arrays.get("/RealOutputs/Alerts/warnings", []):
    for w in arr:
        if w: all_warnings.add(w)

for e in sorted(all_errors):
    p(f"  ❌ ERROR: {e}")
for w in sorted(all_warnings):
    p(f"  ⚠️  WARNING: {w}")
if not all_errors and not all_warnings:
    p("  No alerts")

# ── Summary / Key Findings ───────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  KEY FINDINGS")
p("=" * 80)
findings = []

# Battery
if bv_s and bv_s['min'] < 8.0:
    findings.append(f"CRITICAL: Battery sagged to {bv_s['min']:.2f}V during teleop — risk of brownout/motor damage")
elif bv_s and bv_s['min'] < 10.0:
    findings.append(f"WARNING: Battery sagged to {bv_s['min']:.2f}V — may cause inconsistent motor behavior")

# Loop timing
if loop_s and loop_s['p95'] > 20:
    findings.append(f"CRITICAL: Loop timing p95={loop_s['p95']:.1f}ms (>20ms) — {100*overruns_20/loop_s['n']:.0f}% overruns. Code is too slow for 50Hz.")

if loop_s and loop_s['max'] > 50:
    findings.append(f"WARNING: Worst loop cycle {loop_s['max']:.1f}ms — severe spike that could cause CAN timeouts")

# Vision
if cam_alerts:
    findings.append("WARNING: Vision cameras reported disconnected — check USB/coprocessor")

# Flywheel tracking
if fw_vel and fw_ref and len(fw_vel) == len(fw_ref):
    errors_gt500 = sum(1 for (_, vel), (_, ref) in zip(fw_vel, fw_ref) if abs(ref) > 100 and abs(vel - ref) > 500)
    commanded_n = sum(1 for _, ref in fw_ref if abs(ref) > 100)
    if commanded_n > 0 and errors_gt500 / commanded_n > 0.1:
        findings.append(f"WARNING: Flywheel had >500 RPM error in {100*errors_gt500/commanded_n:.0f}% of commanded samples — may need PID tuning or battery")

# Turret limits
if turret_pos:
    near_limits = sum(1 for _, v in turret_pos if v < -85 or v > 125)
    if near_limits > len(turret_pos) * 0.05:
        findings.append(f"INFO: Turret near mechanical limits {100*near_limits/len(turret_pos):.0f}% of time — consider driving strategy to keep hub in window")

# CAN
if can_s and can_s['max'] > 0.75:
    findings.append(f"WARNING: CAN utilization peaked at {can_s['max']*100:.1f}% — reduce signal update rates if possible")

for i, f in enumerate(findings, 1):
    p(f"  {i}. {f}")

if not findings:
    p("  No critical findings")

p("\n" + "=" * 80)
p(f"  Analysis saved to: {OUTPUT_PATH}")
p("=" * 80)
_out.close()
