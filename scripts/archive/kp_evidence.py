"""
kP Evidence Script — Compare a WORKING match vs a BROKEN match.

Shows chassis velocity setpoints vs measured during auto to demonstrate
the runaway velocity spiral caused by kTranslationP = 5.0 on fast paths.

Q6 = early match, 2.0 m/s path velocity — worked fine.
Q61 = later match, 5.0 m/s path velocity — catastrophic stutter.
"""

import struct as st
import math
from collections import defaultdict

try:
    from wpiutil.log import DataLogReader
except ImportError:
    print("ERROR: pip install robotpy-wpiutil")
    raise

LOG_DIR = r"C:\Users\danla\Desktop\Logs Idaho"

MATCHES = {
    "Q6 (working, 2.0 m/s paths)": {
        "file": f"{LOG_DIR}\\akit_26-03-27_17-41-24_idbo_q6.wpilog",
        "auto_cmd": None,  # will detect auto window from DriverStation
    },
    "Q61 (broken, 5.0 m/s paths)": {
        "file": f"{LOG_DIR}\\akit_26-03-28_16-55-43_idbo_q61.wpilog",
        "auto_cmd": None,
    },
}


def analyze_match(label, log_path):
    print(f"\n{'='*70}")
    print(f"  {label}")
    print(f"  {log_path}")
    print(f"{'='*70}")

    reader = DataLogReader(log_path)

    # Build entry map
    entries = {}
    for rec in reader:
        if rec.isStart():
            d = rec.getStartData()
            entries[d.entry] = (d.name, d.type)

    # Keys we care about
    KEYS = {
        "/RealOutputs/SwerveChassisSpeeds/Setpoints": "struct:ChassisSpeeds",
        "/RealOutputs/SwerveChassisSpeeds/Measured": "struct:ChassisSpeeds",
        "/RealOutputs/SwerveStates/Setpoints": "struct:SwerveModuleState[]",
        "/RealOutputs/SwerveStates/Measured": "struct:SwerveModuleState[]",
        "/DriverStation/Enabled": "boolean",
        "/DriverStation/Autonomous": "boolean",
    }

    eid_map = {}
    for eid, (name, typ) in entries.items():
        if name in KEYS:
            eid_map[eid] = name

    data = defaultdict(list)

    def decode_chassis_speeds(raw_bytes):
        if len(raw_bytes) >= 24:
            vx, vy, omega = st.unpack_from('<ddd', raw_bytes, 0)
            return (vx, vy, omega)
        return None

    def decode_swerve_states(raw_bytes):
        n = len(raw_bytes) // 16
        states = []
        for i in range(n):
            speed, angle = st.unpack_from('<dd', raw_bytes, i * 16)
            states.append((speed, angle))
        return states

    for rec in reader:
        if rec.isStart() or rec.isFinish() or rec.isControl() or rec.isSetMetadata():
            continue
        eid = rec.getEntry()
        if eid not in eid_map:
            continue
        name = eid_map[eid]
        ts = rec.getTimestamp() / 1e6
        try:
            if "ChassisSpeeds" in KEYS[name]:
                raw = bytes(rec.getRaw())
                cs = decode_chassis_speeds(raw)
                if cs:
                    data[name].append((ts, cs))
            elif "SwerveModuleState" in KEYS[name]:
                raw = bytes(rec.getRaw())
                if len(raw) >= 16:
                    states = decode_swerve_states(raw)
                    data[name].append((ts, states))
            elif KEYS[name] == "boolean":
                data[name].append((ts, rec.getBoolean()))
        except:
            pass

    # Find auto window
    en_key = "/DriverStation/Enabled"
    au_key = "/DriverStation/Autonomous"

    auto_start = auto_end = None
    en_ts = data.get(en_key, [])
    au_ts = data.get(au_key, [])

    # Build enabled/auto state timeline
    enabled = False
    autonomous = False
    for ts, val in sorted(en_ts + [(ts, None) for ts, _ in au_ts], key=lambda x: x[0]):
        # Check if this is an enabled record
        for t2, v2 in en_ts:
            if abs(t2 - ts) < 0.001:
                enabled = v2
                break
        for t2, v2 in au_ts:
            if abs(t2 - ts) < 0.001:
                autonomous = v2
                break
        if enabled and autonomous and auto_start is None:
            auto_start = ts
        if auto_start and (not enabled or not autonomous) and auto_end is None:
            auto_end = ts

    if auto_start is None or auto_end is None:
        # Fallback: scan for first enabled+auto transition
        for ts, val in en_ts:
            if val and auto_start is None:
                # Check if autonomous at this time
                closest_auto = min(au_ts, key=lambda x: abs(x[0] - ts), default=None)
                if closest_auto and closest_auto[1]:
                    auto_start = ts
            if auto_start and not val and auto_end is None:
                auto_end = ts
                break

    if auto_start is None:
        print("  ❌ Could not find auto window")
        return None

    if auto_end is None:
        auto_end = auto_start + 15.0  # assume 15s auto

    print(f"\n  Auto window: {auto_start:.2f}s – {auto_end:.2f}s ({auto_end - auto_start:.1f}s)")

    def in_auto(ts):
        return auto_start <= ts <= auto_end

    # ── Chassis speed analysis ──────────────────────────────────
    cs_setp = [(ts, v) for ts, v in data.get("/RealOutputs/SwerveChassisSpeeds/Setpoints", []) if in_auto(ts)]
    cs_meas = [(ts, v) for ts, v in data.get("/RealOutputs/SwerveChassisSpeeds/Measured", []) if in_auto(ts)]

    if not cs_setp or not cs_meas:
        print("  ❌ No chassis speed data")
        return None

    # Velocity setpoint timeline
    print(f"\n  CHASSIS VELOCITY SETPOINTS vs MEASURED (every 0.5s):")
    print(f"  {'Time':>6s}  {'Setp m/s':>10s}  {'Meas m/s':>10s}  {'Error':>8s}  {'Status':>10s}")
    print(f"  {'─'*6}  {'─'*10}  {'─'*10}  {'─'*8}  {'─'*10}")

    ROBOT_MAX = 6.02  # kSpeedAt12Volts

    t = auto_start
    results = []
    while t <= auto_end:
        cs = min(cs_setp, key=lambda x: abs(x[0] - t))
        cm = min(cs_meas, key=lambda x: abs(x[0] - t))
        if abs(cs[0] - t) < 0.25:
            sx, sy, _ = cs[1]
            mx, my, _ = cm[1]
            speed_s = math.hypot(sx, sy)
            speed_m = math.hypot(mx, my)
            err = abs(speed_s - speed_m)
            status = ""
            if speed_s > ROBOT_MAX:
                status = "⚠️ >MAX"
            elif err > 2.0:
                status = "❌ BIG ERR"
            elif err > 0.5:
                status = "⚠️ err"
            else:
                status = "✅ ok"
            print(f"  {t-auto_start:6.1f}s  {speed_s:10.2f}  {speed_m:10.2f}  {err:8.2f}  {status:>10s}")
            results.append((speed_s, speed_m, err))
        t += 0.5

    if results:
        setpoints = [r[0] for r in results]
        measured = [r[1] for r in results]
        errors = [r[2] for r in results]
        peak_setp = max(setpoints)
        avg_err = sum(errors) / len(errors)
        peak_err = max(errors)
        above_max = sum(1 for s in setpoints if s > ROBOT_MAX)

        print(f"\n  SUMMARY:")
        print(f"    Peak setpoint:        {peak_setp:.2f} m/s  (robot max: {ROBOT_MAX:.2f} m/s)")
        print(f"    Setpoints > robot max: {above_max}/{len(setpoints)} samples")
        print(f"    Avg tracking error:   {avg_err:.2f} m/s")
        print(f"    Peak tracking error:  {peak_err:.2f} m/s")

    # ── Module-level setpoint analysis ──────────────────────────
    mod_setp = [(ts, v) for ts, v in data.get("/RealOutputs/SwerveStates/Setpoints", []) if in_auto(ts)]
    if mod_setp:
        mod_names = ["FL", "FR", "BL", "BR"]
        print(f"\n  PER-MODULE PEAK SETPOINTS:")
        for mod in range(min(4, len(mod_setp[0][1]))):
            speeds = [abs(s[1][mod][0]) for s in mod_setp if mod < len(s[1])]
            if speeds:
                peak = max(speeds)
                above = sum(1 for s in speeds if s > ROBOT_MAX)
                print(f"    {mod_names[mod]}: peak={peak:.2f} m/s, above max: {above}/{len(speeds)}")

    return {
        "peak_setp": peak_setp if results else 0,
        "avg_err": avg_err if results else 0,
        "peak_err": peak_err if results else 0,
        "above_max": above_max if results else 0,
    }


# ── Run both matches ────────────────────────────────────────────
results = {}
for label, info in MATCHES.items():
    try:
        r = analyze_match(label, info["file"])
        if r:
            results[label] = r
    except FileNotFoundError:
        print(f"\n  ❌ Log file not found: {info['file']}")
        print(f"     Skipping {label}")

# ── Side-by-side comparison ─────────────────────────────────────
if len(results) == 2:
    print(f"\n\n{'='*70}")
    print("  COMPARISON: WORKING vs BROKEN")
    print(f"{'='*70}")

    labels = list(results.keys())
    r1, r2 = results[labels[0]], results[labels[1]]

    print(f"\n  {'Metric':<30s}  {'Working':>12s}  {'Broken':>12s}")
    print(f"  {'─'*30}  {'─'*12}  {'─'*12}")
    print(f"  {'Peak velocity setpoint':<30s}  {r1['peak_setp']:>10.2f} m/s  {r2['peak_setp']:>10.2f} m/s")
    print(f"  {'Setpoints > 6.02 m/s':<30s}  {r1['above_max']:>12d}  {r2['above_max']:>12d}")
    print(f"  {'Avg tracking error':<30s}  {r1['avg_err']:>10.2f} m/s  {r2['avg_err']:>10.2f} m/s")
    print(f"  {'Peak tracking error':<30s}  {r1['peak_err']:>10.2f} m/s  {r2['peak_err']:>10.2f} m/s")

    print(f"\n  kTranslationP was 5.0 in BOTH matches.")
    print(f"  Path velocity changed: 2.0 m/s (working) → 5.0 m/s (broken).")
    print(f"  Robot physical max: 6.02 m/s (kSpeedAt12Volts).")
    print()
    print(f"  THE MATH (broken match, 1m position error):")
    print(f"    PP path velocity:     5.0 m/s")
    print(f"    + kP × error:         5.0 × 1.0 = 5.0 m/s correction")
    print(f"    = Total demand:       10.0 m/s  ← physically impossible")
    print(f"    Robot max:            6.02 m/s")
    print(f"    → desaturateWheelSpeeds distorts module commands")
    print(f"    → robot drives wrong direction → MORE position error → repeat")
    print()
    print(f"  THE MATH (working match, same 1m error):")
    print(f"    PP path velocity:     2.0 m/s")
    print(f"    + kP × error:         5.0 × 1.0 = 5.0 m/s correction")
    print(f"    = Total demand:       7.0 m/s  ← close to limit but mostly ok")
    print(f"    Most of the time error is <0.5m → 2.0 + 2.5 = 4.5 m/s ✅")
    print()
    print(f"  CURRENT FIX (kP=2.5, paths at 3.0 m/s):")
    print(f"    PP path velocity:     3.0 m/s")
    print(f"    + kP × error:         2.5 × 1.0 = 2.5 m/s correction")
    print(f"    = Total demand:       5.5 m/s  ← safely under 6.02 m/s ✅")
    print(f"    Typical (0.3m error): 3.0 + 0.75 = 3.75 m/s ✅")

print("\nDone.")
