"""
MRT3216 — Idaho Log Analyzer
Quick single-log analysis: battery, currents, shooter pipeline, turret, hood,
hub shifts, vision, loop timing, commands, CRT, alerts.

Usage:
    .venv/Scripts/python.exe scripts/analyze_idaho.py <path-to-wpilog>
"""

import sys, os, bisect
from collections import defaultdict
from wpiutil.log import DataLogReader

if len(sys.argv) < 2:
    print("Usage: python analyze_idaho.py <path-to-wpilog>")
    sys.exit(1)

LOG_PATH = sys.argv[1]
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PATH = os.path.join(SCRIPT_DIR, "idaho_analysis.txt")
_out = open(OUTPUT_PATH, "w", encoding="utf-8")

def p(*args, **kwargs):
    print(*args, **kwargs)
    print(*args, **kwargs, file=_out)
    _out.flush()

# ── Signals of interest ─────────────────────────────────────────────────────
SIGNALS = {
    # State
    "/DriverStation/Enabled":                       "boolean",
    "/DriverStation/Autonomous":                    "boolean",
    "/DriverStation/MatchTime":                     "double",
    "/DriverStation/MatchNumber":                   "int64",
    "/DriverStation/MatchType":                     "int64",
    # Battery
    "/SystemStats/BatteryVoltage":                  "double",
    "/SystemStats/BatteryCurrent":                  "double",
    "/SystemStats/BrownedOut":                      "boolean",
    "/SystemStats/CPUTempCelsius":                  "double",
    "/SystemStats/CANBus/Utilization":              "float",
    "/SystemStats/CANBus/OffCount":                 "int64",
    # Loop
    "/RealOutputs/LoggedRobot/FullCycleMS":         "double",
    "/RealOutputs/LoggedRobot/UserCodeMS":          "double",
    "/RealOutputs/Logger/QueuedCycles":             "int64",
    # Subsystem currents
    "/Shooter/Flywheel/Current":                    "double",
    "/Shooter/Flywheel/Velocity":                   "double",
    "/Shooter/Flywheel/Setpoint":                   "double",
    "/Hood/Current":                                "double",
    "/Hood/Angle":                                  "double",
    "/Hood/Setpoint":                               "double",
    "/Kicker/Current":                              "double",
    "/Kicker/Velocity":                             "double",
    "/Kicker/Setpoint":                             "double",
    "/Spindexer/Current":                           "double",
    "/Spindexer/MechanismVelocity":                 "double",
    "/Spindexer/Setpoint":                          "double",
    "/Intake/Pivot/Velocity":                       "double",
    "/Intake/Rollers/Velocity":                     "double",
    # Turret
    "/Shooter/Turret/Current":                      "double",
    "/Shooter/Turret/Angle":                        "double",
    "/Shooter/Turret/Setpoint":                     "double",
    "/RealOutputs/Shooter/Turret/PositionDegrees":  "double",
    # CRT
    "/RealOutputs/Shooter/Turret/CRT/Status":       "string",
    "/RealOutputs/Shooter/Turret/CRT/ErrorRot":     "double",
    "/RealOutputs/Shooter/Turret/CRT/ResolvedDeg":  "double",
    "/RealOutputs/Shooter/Turret/CRT/AttemptsUsed": "int64",
    # Flywheel FX
    "/RealOutputs/Flywheel/FX/VelocityRPM":         "double",
    "/RealOutputs/Flywheel/FX/ReferenceRPM":        "double",
    "/RealOutputs/Flywheel/IsSpunUp":                "boolean",
    # Hood FX
    "/RealOutputs/Hood/FX/PositionDegrees":          "double",
    "/RealOutputs/Hood/FX/ReferenceDegrees":         "double",
    # HubShift
    "/RealOutputs/HubShift/Active":                  "boolean",
    "/RealOutputs/HubShift/ShiftedActive":           "boolean",
    "/RealOutputs/HubShift/CurrentShift":            "string",
    "/RealOutputs/HubShift/RemainingTime":           "double",
    # Hybrid aiming
    "/RealOutputs/HybridAiming/aimEnabled":          "boolean",
    "/RealOutputs/HybridAiming/rawTurretAzimuthDeg": "double",
    "/RealOutputs/HybridAiming/clampedTurretAzimuthDeg": "double",
    "/RealOutputs/HybridAiming/turretClamped":       "boolean",
    "/RealOutputs/HybridAiming/rampFactor":          "double",
    "/RealOutputs/HybridAiming/outsideTravelWindow": "boolean",
    # Shooter telemetry
    "/RealOutputs/ShooterTelemetry/fudgedRPM":       "double",
    "/RealOutputs/ShooterTelemetry/hubDistanceMeters":"double",
    "/RealOutputs/ShooterTelemetry/modelRPM":        "double",
    "/RealOutputs/ShooterTelemetry/rpmFudgeRPM":     "double",
    "/RealOutputs/ShooterTelemetry/shootMode":        "string",
    "/RealOutputs/ShooterTelemetry/lutHoodDegrees":   "double",
    "/RealOutputs/ShooterTelemetry/lutToFSeconds":    "double",
    "/RealOutputs/ShooterTelemetry/isValid":          "boolean",
    # Commands
    "/RealOutputs/CommandsAll/HybridAimAndShoot":    "boolean",
    "/RealOutputs/CommandsAll/HybridPassShoot":      "boolean",
    "/RealOutputs/CommandsAll/Intake.DutyCycleIntake":"boolean",
    "/RealOutputs/CommandsAll/Intake.DutyCycleEject": "boolean",
    "/RealOutputs/CommandsAll/Intake.DutyCycleAgitate":"boolean",
    "/RealOutputs/CommandsAll/Left Bum Rush (No SOTF)":"boolean",
    "/RealOutputs/CommandsAll/FlywheelStopHold":      "boolean",
    # Alerts
    "/RealOutputs/Alerts/errors":                     "string[]",
    "/RealOutputs/Alerts/warnings":                   "string[]",
    # Energy
    "/RealOutputs/EnergyLogger/TotalEnergyJoules":    "double",
    "/RealOutputs/EnergyLogger/TotalPowerWatts":      "double",
    "/RealOutputs/EnergyLogger/TotalCurrentAmps":     "double",
    # Vision
    "/RealOutputs/Vision/Summary/HasTarget":          "boolean",
    "/RealOutputs/Vision/Summary/TagCount":           "int64",
    # Drive currents (via EnergyLogger)
    "/Drive/Module0/DriveCurrentAmps":                "double",
    "/Drive/Module0/TurnCurrentAmps":                 "double",
    "/Drive/Module1/DriveCurrentAmps":                "double",
    "/Drive/Module1/TurnCurrentAmps":                 "double",
    "/Drive/Module2/DriveCurrentAmps":                "double",
    "/Drive/Module2/TurnCurrentAmps":                 "double",
    "/Drive/Module3/DriveCurrentAmps":                "double",
    "/Drive/Module3/TurnCurrentAmps":                 "double",
}

# ── Parse ────────────────────────────────────────────────────────────────────
entry_map = {}
type_map = {}
raw = defaultdict(list)
raw_t = defaultdict(list)
string_data = defaultdict(list)  # for string/string[] signals

p(f"Parsing {LOG_PATH}...")

for record in DataLogReader(LOG_PATH):
    if record.isStart():
        d = record.getStartData()
        if d.name in SIGNALS:
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
            val = record.getDouble()
            raw[name].append(val)
            raw_t[name].append(t)
        elif typ == "boolean":
            val = 1.0 if record.getBoolean() else 0.0
            raw[name].append(val)
            raw_t[name].append(t)
        elif typ == "int64":
            val = float(record.getInteger())
            raw[name].append(val)
            raw_t[name].append(t)
        elif typ == "string":
            val = record.getString()
            string_data[name].append((t, val))
        elif typ == "string[]":
            val = record.getStringArray()
            string_data[name].append((t, list(val)))
    except Exception:
        pass

# ── Teleop window detection ─────────────────────────────────────────────────
enabled_pts = list(zip(raw_t["/DriverStation/Enabled"], raw["/DriverStation/Enabled"]))
auton_pts = list(zip(raw_t["/DriverStation/Autonomous"], raw["/DriverStation/Autonomous"]))

all_change_times = sorted(set(t for t, _ in enabled_pts) | set(t for t, _ in auton_pts))
en_sorted = sorted(enabled_pts)
au_sorted = sorted(auton_pts)
tele_times, tele_vals = [], []
en = au = 0.0
ei = ai = 0
for ct in all_change_times:
    while ei < len(en_sorted) and en_sorted[ei][0] <= ct:
        en = en_sorted[ei][1]; ei += 1
    while ai < len(au_sorted) and au_sorted[ai][0] <= ct:
        au = au_sorted[ai][1]; ai += 1
    tele_times.append(ct)
    tele_vals.append(en == 1.0 and au == 0.0)

# Also detect auto windows
auto_times, auto_vals = [], []
en2 = au2 = 0.0
ei2 = ai2 = 0
for ct in all_change_times:
    while ei2 < len(en_sorted) and en_sorted[ei2][0] <= ct:
        en2 = en_sorted[ei2][1]; ei2 += 1
    while ai2 < len(au_sorted) and au_sorted[ai2][0] <= ct:
        au2 = au_sorted[ai2][1]; ai2 += 1
    auto_times.append(ct)
    auto_vals.append(en2 == 1.0 and au2 == 1.0)

def in_teleop(t):
    idx = bisect.bisect_right(tele_times, t) - 1
    return idx >= 0 and tele_vals[idx]

def in_auto(t):
    idx = bisect.bisect_right(auto_times, t) - 1
    return idx >= 0 and auto_vals[idx]

def in_enabled(t):
    return in_teleop(t) or in_auto(t)

def filter_teleop(name):
    if name not in raw:
        return []
    return [(t, v) for t, v in zip(raw_t[name], raw[name]) if in_teleop(t)]

def filter_enabled(name):
    if name not in raw:
        return []
    return [(t, v) for t, v in zip(raw_t[name], raw[name]) if in_enabled(t)]

def stats(pairs):
    if not pairs:
        return None
    vals = [v for _, v in pairs]
    return {"n": len(vals), "min": min(vals), "max": max(vals), "avg": sum(vals)/len(vals)}

# ── Timing info ──────────────────────────────────────────────────────────────
log_start = min(raw_t["/DriverStation/Enabled"]) if raw_t["/DriverStation/Enabled"] else 0
log_end = max(raw_t["/DriverStation/Enabled"]) if raw_t["/DriverStation/Enabled"] else 0

# Find auto and teleop boundaries
auto_start = auto_end = teleop_start = teleop_end = None
for t, v in sorted(enabled_pts):
    idx = bisect.bisect_right(auto_times, t) - 1
    is_auto = idx >= 0 and auto_vals[idx]
    is_en = v == 1.0
    if is_en and is_auto and auto_start is None:
        auto_start = t
    if is_en and is_auto:
        auto_end = t
    if is_en and not is_auto and teleop_start is None and auto_end is not None:
        teleop_start = t
    if is_en and not is_auto:
        teleop_end = t

# ── Report ───────────────────────────────────────────────────────────────────
p("=" * 80)
p("  MRT3216 IDAHO LOG ANALYSIS")
p(f"  File: {os.path.basename(LOG_PATH)}")
p(f"  Size: {os.path.getsize(LOG_PATH) / 1e6:.1f} MB")
p(f"  Log duration: {log_end - log_start:.1f}s")
if auto_start:
    p(f"  Auto: {auto_start:.1f}s - {auto_end:.1f}s ({auto_end - auto_start:.1f}s)")
if teleop_start:
    p(f"  Teleop: {teleop_start:.1f}s - {teleop_end:.1f}s ({teleop_end - teleop_start:.1f}s)")
p("=" * 80)

# Match info
match_num = raw.get("/DriverStation/MatchNumber", [])
match_type = raw.get("/DriverStation/MatchType", [])
if match_num:
    p(f"\nMatch Number: {int(match_num[-1])}")
if match_type:
    mt = int(match_type[-1])
    mt_str = {0: "None", 1: "Practice", 2: "Qualification", 3: "Elimination"}.get(mt, f"Unknown({mt})")
    p(f"Match Type: {mt_str}")

# ── CRT Status ───────────────────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  CRT ENCODER STATUS")
p("=" * 80)
crt_status = string_data.get("/RealOutputs/Shooter/Turret/CRT/Status", [])
if crt_status:
    for t, s in crt_status[:5]:
        p(f"  t={t:.2f}s  Status={s}")
    crt_error = raw.get("/RealOutputs/Shooter/Turret/CRT/ErrorRot", [])
    crt_resolved = raw.get("/RealOutputs/Shooter/Turret/CRT/ResolvedDeg", [])
    crt_attempts = raw.get("/RealOutputs/Shooter/Turret/CRT/AttemptsUsed", [])
    if crt_error:
        p(f"  Error (rot): {crt_error[0]:.6f}")
    if crt_resolved:
        p(f"  Resolved angle: {crt_resolved[0]:.2f} deg")
    if crt_attempts:
        p(f"  Attempts used: {int(crt_attempts[0])}")
else:
    p("  NO CRT DATA")

# ── Battery ──────────────────────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  BATTERY / POWER")
p("=" * 80)
bv_tele = filter_teleop("/SystemStats/BatteryVoltage")
bc_tele = filter_teleop("/SystemStats/BatteryCurrent")
bv_s = stats(bv_tele)
bc_s = stats(bc_tele)
if bv_s:
    p(f"  Voltage (teleop): avg={bv_s['avg']:.2f}V  min={bv_s['min']:.2f}V  max={bv_s['max']:.2f}V")
if bc_s:
    p(f"  Current (teleop): avg={bc_s['avg']:.1f}A  max={bc_s['max']:.1f}A")

# Brownout check
brownout = filter_teleop("/SystemStats/BrownedOut")
brownout_count = sum(1 for _, v in brownout if v == 1.0)
if brownout_count > 0:
    p(f"  *** BROWNOUT DETECTED: {brownout_count} samples ***")
else:
    p("  No brownouts detected")

# Voltage sag
if bv_tele:
    low_v = [(t, v) for t, v in bv_tele if v < 8.0]
    if low_v:
        worst = min(low_v, key=lambda x: x[1])
        p(f"  *** {len(low_v)} samples below 8V! Worst: {worst[1]:.2f}V at t={worst[0]:.1f}s ***")

    low_10 = sorted(bv_tele, key=lambda x: x[1])[:10]
    p("  10 lowest voltage readings:")
    for t, v in low_10:
        p(f"    t={t:.1f}s  {v:.3f}V")

# Total energy
energy = raw.get("/RealOutputs/EnergyLogger/TotalEnergyJoules", [])
if energy:
    p(f"  Total energy consumed: {energy[-1]:.0f} J ({energy[-1]/3600:.1f} Wh)")

# CPU temp
cpu = filter_teleop("/SystemStats/CPUTempCelsius")
cpu_s = stats(cpu)
if cpu_s:
    p(f"  CPU temp (teleop): avg={cpu_s['avg']:.1f}C  max={cpu_s['max']:.1f}C")

# CAN bus
can_util = filter_teleop("/SystemStats/CANBus/Utilization")
can_s = stats(can_util)
if can_s:
    p(f"  CAN utilization (teleop): avg={can_s['avg']*100:.1f}%  max={can_s['max']*100:.1f}%")

can_off = raw.get("/SystemStats/CANBus/OffCount", [])
if can_off:
    p(f"  CAN off count: {int(max(can_off))}")

# ── Loop Timing ──────────────────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  LOOP TIMING")
p("=" * 80)
cycle = filter_enabled("/RealOutputs/LoggedRobot/FullCycleMS")
user = filter_enabled("/RealOutputs/LoggedRobot/UserCodeMS")
cycle_s = stats(cycle)
user_s = stats(user)
if cycle_s:
    p(f"  Full cycle (enabled): avg={cycle_s['avg']:.2f}ms  max={cycle_s['max']:.2f}ms")
    overruns = sum(1 for _, v in cycle if v > 20)
    pct = 100 * overruns / cycle_s['n'] if cycle_s['n'] else 0
    p(f"  Loop overruns (>20ms): {overruns} ({pct:.1f}%)")
if user_s:
    p(f"  User code (enabled): avg={user_s['avg']:.2f}ms  max={user_s['max']:.2f}ms")

queued = filter_enabled("/RealOutputs/Logger/QueuedCycles")
q_s = stats(queued)
if q_s:
    p(f"  Logger queued cycles: avg={q_s['avg']:.1f}  max={int(q_s['max'])}")

# ── Drive Currents ───────────────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  DRIVE CURRENTS (teleop)")
p("=" * 80)
total_drive = []
for i in range(4):
    d = filter_teleop(f"/Drive/Module{i}/DriveCurrentAmps")
    t = filter_teleop(f"/Drive/Module{i}/TurnCurrentAmps")
    d_s = stats(d)
    t_s = stats(t)
    if d_s:
        p(f"  Module {i} Drive: avg={d_s['avg']:.1f}A  max={d_s['max']:.1f}A")
        total_drive.extend([v for _, v in d])
    if t_s:
        p(f"  Module {i} Turn:  avg={t_s['avg']:.1f}A  max={t_s['max']:.1f}A")
if total_drive:
    p(f"  Total drive avg: {sum(total_drive)/len(total_drive):.1f}A  max: {max(total_drive):.1f}A")

# ── Shooter Pipeline ─────────────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  SHOOTER PIPELINE")
p("=" * 80)

# Flywheel
fw_vel = filter_teleop("/RealOutputs/Flywheel/FX/VelocityRPM")
fw_ref = filter_teleop("/RealOutputs/Flywheel/FX/ReferenceRPM")
fw_cur = filter_teleop("/Shooter/Flywheel/Current")
fw_vel_s = stats(fw_vel)
fw_ref_s = stats(fw_ref)
fw_cur_s = stats(fw_cur)
p("  Flywheel:")
if fw_vel_s:
    p(f"    Velocity: avg={fw_vel_s['avg']:.0f} RPM  max={fw_vel_s['max']:.0f} RPM")
if fw_ref_s:
    p(f"    Reference: avg={fw_ref_s['avg']:.0f} RPM  max={fw_ref_s['max']:.0f} RPM")
if fw_cur_s:
    p(f"    Current: avg={fw_cur_s['avg']:.1f}A  max={fw_cur_s['max']:.1f}A")

# Spun up
spun = filter_teleop("/RealOutputs/Flywheel/IsSpunUp")
if spun:
    spun_count = sum(1 for _, v in spun if v == 1.0)
    p(f"    Spun up: {spun_count}/{len(spun)} samples ({100*spun_count/len(spun):.1f}%)")

# Flywheel RPM error during shooting
fw_vel_pairs = [(t, v) for t, v in fw_vel]
fw_ref_pairs = [(t, v) for t, v in fw_ref]
if fw_vel_pairs and fw_ref_pairs:
    # Only when reference > 100 RPM (actively shooting)
    active_errors = []
    for (t, vel), (_, ref) in zip(fw_vel_pairs, fw_ref_pairs):
        if abs(ref) > 100:
            active_errors.append(abs(vel - ref))
    if active_errors:
        avg_err = sum(active_errors) / len(active_errors)
        max_err = max(active_errors)
        p(f"    RPM error (when active): avg={avg_err:.0f}  max={max_err:.0f}")

# Hood
p("  Hood:")
hood_pos = filter_teleop("/RealOutputs/Hood/FX/PositionDegrees")
hood_ref = filter_teleop("/RealOutputs/Hood/FX/ReferenceDegrees")
hood_cur = filter_teleop("/Hood/Current")
if hood_pos:
    p(f"    Position: avg={stats(hood_pos)['avg']:.1f}deg  max={stats(hood_pos)['max']:.1f}deg")
if hood_ref:
    p(f"    Reference: avg={stats(hood_ref)['avg']:.1f}deg  max={stats(hood_ref)['max']:.1f}deg")
if hood_cur:
    p(f"    Current: avg={stats(hood_cur)['avg']:.1f}A  max={stats(hood_cur)['max']:.1f}A")

# Kicker
p("  Kicker:")
k_cur = filter_teleop("/Kicker/Current")
k_vel = filter_teleop("/Kicker/Velocity")
if k_cur:
    p(f"    Current: avg={stats(k_cur)['avg']:.1f}A  max={stats(k_cur)['max']:.1f}A")
if k_vel:
    active_k = [(t, v) for t, v in k_vel if abs(v) > 0.1]
    p(f"    Active samples: {len(active_k)}/{len(k_vel)}")

# Spindexer
p("  Spindexer:")
sp_cur = filter_teleop("/Spindexer/Current")
sp_vel = filter_teleop("/Spindexer/MechanismVelocity")
if sp_cur:
    p(f"    Current: avg={stats(sp_cur)['avg']:.1f}A  max={stats(sp_cur)['max']:.1f}A")
if sp_vel:
    active_sp = [(t, v) for t, v in sp_vel if abs(v) > 0.1]
    p(f"    Active samples: {len(active_sp)}/{len(sp_vel)}")

# ── Turret ───────────────────────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  TURRET")
p("=" * 80)
turret_pos = filter_teleop("/RealOutputs/Shooter/Turret/PositionDegrees")
turret_cur = filter_teleop("/Shooter/Turret/Current")
if turret_pos:
    t_s = stats(turret_pos)
    p(f"  Position: avg={t_s['avg']:.1f}deg  min={t_s['min']:.1f}deg  max={t_s['max']:.1f}deg")
if turret_cur:
    tc_s = stats(turret_cur)
    p(f"  Current: avg={tc_s['avg']:.1f}A  max={tc_s['max']:.1f}A")

# ── Hybrid Aiming ────────────────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  HYBRID AIMING")
p("=" * 80)
aim_en = filter_teleop("/RealOutputs/HybridAiming/aimEnabled")
if aim_en:
    aim_count = sum(1 for _, v in aim_en if v == 1.0)
    p(f"  Aim enabled: {aim_count}/{len(aim_en)} samples ({100*aim_count/len(aim_en):.1f}%)")

raw_az = filter_teleop("/RealOutputs/HybridAiming/rawTurretAzimuthDeg")
clamp_az = filter_teleop("/RealOutputs/HybridAiming/clampedTurretAzimuthDeg")
turret_clamped = filter_teleop("/RealOutputs/HybridAiming/turretClamped")
outside_win = filter_teleop("/RealOutputs/HybridAiming/outsideTravelWindow")

if raw_az:
    r_s = stats(raw_az)
    p(f"  Raw azimuth: min={r_s['min']:.1f}deg  max={r_s['max']:.1f}deg")
if clamp_az:
    c_s = stats(clamp_az)
    p(f"  Clamped azimuth: min={c_s['min']:.1f}deg  max={c_s['max']:.1f}deg")
if turret_clamped:
    clamped_count = sum(1 for _, v in turret_clamped if v == 1.0)
    p(f"  Turret clamped: {clamped_count}/{len(turret_clamped)} ({100*clamped_count/len(turret_clamped):.1f}%)")
if outside_win:
    outside_count = sum(1 for _, v in outside_win if v == 1.0)
    p(f"  Outside travel window: {outside_count}/{len(outside_win)} ({100*outside_count/len(outside_win):.1f}%)")

ramp = filter_teleop("/RealOutputs/HybridAiming/rampFactor")
if ramp:
    ramp_s = stats(ramp)
    p(f"  Heading ramp factor: avg={ramp_s['avg']:.2f}  max={ramp_s['max']:.2f}")

# ── Hub Shift ────────────────────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  HUB SHIFT")
p("=" * 80)
shifted_active = filter_teleop("/RealOutputs/HubShift/ShiftedActive")
if shifted_active:
    active_count = sum(1 for _, v in shifted_active if v == 1.0)
    p(f"  Shifted-active: {active_count}/{len(shifted_active)} ({100*active_count/len(shifted_active):.1f}%)")

shift_str = [(t, s) for t, s in string_data.get("/RealOutputs/HubShift/CurrentShift", []) if in_teleop(t)]
if shift_str:
    # Show shift transitions
    prev = None
    p("  Shift transitions:")
    for t, s in shift_str:
        if s != prev:
            p(f"    t={t:.1f}s  -> {s}")
            prev = s

# ── Shooter Telemetry ────────────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  SHOOTER TELEMETRY (during aim-and-shoot)")
p("=" * 80)
hub_dist = filter_teleop("/RealOutputs/ShooterTelemetry/hubDistanceMeters")
model_rpm = filter_teleop("/RealOutputs/ShooterTelemetry/modelRPM")
fudged_rpm = filter_teleop("/RealOutputs/ShooterTelemetry/fudgedRPM")
rpm_fudge = filter_teleop("/RealOutputs/ShooterTelemetry/rpmFudgeRPM")
lut_hood = filter_teleop("/RealOutputs/ShooterTelemetry/lutHoodDegrees")
is_valid = filter_teleop("/RealOutputs/ShooterTelemetry/isValid")

if hub_dist:
    h_s = stats(hub_dist)
    p(f"  Hub distance: avg={h_s['avg']:.2f}m  min={h_s['min']:.2f}m  max={h_s['max']:.2f}m")
if model_rpm:
    m_s = stats(model_rpm)
    p(f"  Model RPM: avg={m_s['avg']:.0f}  min={m_s['min']:.0f}  max={m_s['max']:.0f}")
if fudged_rpm:
    f_s = stats(fudged_rpm)
    p(f"  Fudged RPM: avg={f_s['avg']:.0f}  min={f_s['min']:.0f}  max={f_s['max']:.0f}")
if rpm_fudge:
    rf_s = stats(rpm_fudge)
    p(f"  RPM fudge offset: avg={rf_s['avg']:.0f}  min={rf_s['min']:.0f}  max={rf_s['max']:.0f}")
if lut_hood:
    lh_s = stats(lut_hood)
    p(f"  LUT hood angle: avg={lh_s['avg']:.1f}deg  min={lh_s['min']:.1f}deg  max={lh_s['max']:.1f}deg")
if is_valid:
    valid_count = sum(1 for _, v in is_valid if v == 1.0)
    p(f"  Solution valid: {valid_count}/{len(is_valid)} ({100*valid_count/len(is_valid):.1f}%)")

shoot_mode = [(t, s) for t, s in string_data.get("/RealOutputs/ShooterTelemetry/shootMode", []) if in_teleop(t)]
if shoot_mode:
    prev = None
    p("  Shoot mode changes:")
    for t, s in shoot_mode:
        if s != prev:
            p(f"    t={t:.1f}s  -> {s}")
            prev = s

# ── Commands Timeline ────────────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  COMMAND ACTIVITY (teleop)")
p("=" * 80)
cmd_signals = [
    ("/RealOutputs/CommandsAll/HybridAimAndShoot", "HybridAimAndShoot"),
    ("/RealOutputs/CommandsAll/HybridPassShoot", "HybridPassShoot"),
    ("/RealOutputs/CommandsAll/Intake.DutyCycleIntake", "Intake"),
    ("/RealOutputs/CommandsAll/Intake.DutyCycleEject", "Eject"),
    ("/RealOutputs/CommandsAll/Intake.DutyCycleAgitate", "Agitate"),
    ("/RealOutputs/CommandsAll/Left Bum Rush (No SOTF)", "Auto: BumRush"),
]
for sig, label in cmd_signals:
    pts = filter_teleop(sig) if "Auto" not in label else filter_enabled(sig)
    if pts:
        active = sum(1 for _, v in pts if v == 1.0)
        total_time = len(pts) * 0.02  # ~50Hz
        active_time = active * 0.02
        p(f"  {label:<25} active: {active_time:.1f}s / {total_time:.1f}s ({100*active/len(pts):.1f}%)")
    else:
        p(f"  {label:<25} NO DATA")

# ── Vision ───────────────────────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  VISION")
p("=" * 80)
has_target = filter_teleop("/RealOutputs/Vision/Summary/HasTarget")
tag_count = filter_teleop("/RealOutputs/Vision/Summary/TagCount")
if has_target:
    target_count = sum(1 for _, v in has_target if v == 1.0)
    p(f"  Has target: {target_count}/{len(has_target)} ({100*target_count/len(has_target):.1f}%)")
if tag_count:
    tc_s = stats(tag_count)
    p(f"  Tag count: avg={tc_s['avg']:.1f}  max={int(tc_s['max'])}")

# ── Alerts ───────────────────────────────────────────────────────────────────
p("\n" + "=" * 80)
p("  ALERTS")
p("=" * 80)
errors = string_data.get("/RealOutputs/Alerts/errors", [])
warnings = string_data.get("/RealOutputs/Alerts/warnings", [])

# Show unique non-empty alerts
seen_errors = set()
seen_warnings = set()
for t, arr in errors:
    for e in arr:
        if e and e not in seen_errors:
            seen_errors.add(e)
            p(f"  ERROR: {e}")
for t, arr in warnings:
    for w in arr:
        if w and w not in seen_warnings:
            seen_warnings.add(w)
            p(f"  WARNING: {w}")
if not seen_errors and not seen_warnings:
    p("  No alerts logged")

p("\n" + "=" * 80)
p(f"  Analysis complete. Output saved to: {OUTPUT_PATH}")
p("=" * 80)

_out.close()
