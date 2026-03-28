"""
Scan ALL 12 match logs for auto velocity setpoint vs measured.
Shows whether the stuttering (setpoint > robot max, large tracking error)
was present in early matches or only appeared after the path velocity bump.
"""

import struct as st
import math, os
from collections import defaultdict

try:
    from wpiutil.log import DataLogReader
except ImportError:
    print("ERROR: pip install robotpy-wpiutil")
    raise

LOG_DIR = r"C:\Users\danla\Desktop\Logs Idaho"

# All match logs in chronological order
LOGS = [
    ("Q6",  "akit_26-03-27_17-41-24_idbo_q6.wpilog"),
    ("Q12", "akit_26-03-27_18-36-09_idbo_q12.wpilog"),
    ("Q17", "akit_26-03-27_20-16-34_idbo_q17.wpilog"),
    ("Q23", "akit_26-03-27_21-18-04_idbo_q23.wpilog"),
    ("Q26", "akit_26-03-27_21-49-19_idbo_q26.wpilog"),
    ("Q32", "akit_26-03-27_22-47-39_idbo_q32.wpilog"),
    ("Q39", "akit_26-03-27_23-53-46_idbo_q39.wpilog"),
    ("Q46", "akit_26-03-28_01-01-39_idbo_q46.wpilog"),
    ("Q52", "akit_26-03-28_15-30-27_idbo_q52.wpilog"),
    ("Q55", "akit_26-03-28_15-58-15_idbo_q55.wpilog"),
    ("Q61", "akit_26-03-28_16-55-43_idbo_q61.wpilog"),
    ("Q67", "akit_26-03-28_17-53-34_idbo_q67.wpilog"),
]

# Path velocity timeline from git:
# Q6: 2.0 m/s  (commit 9ffd86e, 3/26 18:59)
# Q12-Q46: 5.0 m/s  (commits a1b1bf2+5e8ad03, 3/27 12:20-13:31)
# Q52-Q67: 4.0 m/s  (commit 3961e10, 3/28 09:13 "Standardized")
PATH_VEL = {
    "Q6": 2.0,
    "Q12": 5.0, "Q17": 5.0, "Q23": 5.0, "Q26": 5.0, "Q32": 5.0, "Q39": 5.0, "Q46": 5.0,
    "Q52": 4.0, "Q55": 4.0, "Q61": 4.0, "Q67": 4.0,
}

ROBOT_MAX = 6.02


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


def analyze_match(label, log_path):
    if not os.path.exists(log_path):
        return None

    reader = DataLogReader(log_path)

    entries = {}
    for rec in reader:
        if rec.isStart():
            d = rec.getStartData()
            entries[d.entry] = (d.name, d.type)

    KEYS = {
        "/RealOutputs/SwerveChassisSpeeds/Setpoints": "struct:ChassisSpeeds",
        "/RealOutputs/SwerveChassisSpeeds/Measured": "struct:ChassisSpeeds",
        "/RealOutputs/SwerveStates/Setpoints": "struct:SwerveModuleState[]",
        "/DriverStation/Enabled": "boolean",
        "/DriverStation/Autonomous": "boolean",
    }

    eid_map = {}
    for eid, (name, typ) in entries.items():
        if name in KEYS:
            eid_map[eid] = name

    data = defaultdict(list)

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

    # Find auto window from DriverStation
    en_data = data.get("/DriverStation/Enabled", [])
    au_data = data.get("/DriverStation/Autonomous", [])

    if not en_data or not au_data:
        return None

    # Build timeline
    auto_start = auto_end = None
    # Merge and sort all DS events
    events = []
    for ts, val in en_data:
        events.append((ts, 'en', val))
    for ts, val in au_data:
        events.append((ts, 'au', val))
    events.sort()

    enabled = False
    autonomous = False
    for ts, key, val in events:
        if key == 'en':
            enabled = val
        elif key == 'au':
            autonomous = val
        if enabled and autonomous and auto_start is None:
            auto_start = ts
        if auto_start is not None and (not enabled or not autonomous) and auto_end is None:
            auto_end = ts

    if auto_start is None:
        return None
    if auto_end is None:
        auto_end = auto_start + 15.0

    def in_auto(ts):
        return auto_start <= ts <= auto_end

    auto_dur = auto_end - auto_start

    # Chassis speed analysis
    cs_setp = [(ts, v) for ts, v in data.get("/RealOutputs/SwerveChassisSpeeds/Setpoints", []) if in_auto(ts)]
    cs_meas = [(ts, v) for ts, v in data.get("/RealOutputs/SwerveChassisSpeeds/Measured", []) if in_auto(ts)]

    if not cs_setp or not cs_meas:
        return None

    # Sample every 0.25s for finer resolution
    results = []
    t = auto_start
    while t <= auto_end:
        cs = min(cs_setp, key=lambda x: abs(x[0] - t))
        cm = min(cs_meas, key=lambda x: abs(x[0] - t))
        if abs(cs[0] - t) < 0.2:
            sx, sy, _ = cs[1]
            mx, my, _ = cm[1]
            speed_s = math.hypot(sx, sy)
            speed_m = math.hypot(mx, my)
            err = abs(speed_s - speed_m)
            results.append((t - auto_start, speed_s, speed_m, err))
        t += 0.25

    if not results:
        return None

    setpoints = [r[1] for r in results]
    errors = [r[3] for r in results]
    peak_setp = max(setpoints)
    avg_setp = sum(setpoints) / len(setpoints)
    avg_err = sum(errors) / len(errors)
    peak_err = max(errors)
    above_max = sum(1 for s in setpoints if s > ROBOT_MAX)
    big_err_count = sum(1 for e in errors if e > 2.0)

    # Module-level: check for per-module setpoints above max
    mod_setp = [(ts, v) for ts, v in data.get("/RealOutputs/SwerveStates/Setpoints", []) if in_auto(ts)]
    mod_above = 0
    mod_total = 0
    if mod_setp:
        for ts, states in mod_setp:
            for mod in range(min(4, len(states))):
                mod_total += 1
                if abs(states[mod][0]) > ROBOT_MAX:
                    mod_above += 1

    # Detect "stutter" periods: >2s continuous where error > 1.0 m/s
    stutter_time = 0
    in_stutter = False
    stutter_start = None
    stutters = []
    for t_rel, sp, ms, err in results:
        if err > 1.0:
            if not in_stutter:
                in_stutter = True
                stutter_start = t_rel
        else:
            if in_stutter:
                dur = t_rel - stutter_start
                if dur >= 0.5:
                    stutters.append((stutter_start, t_rel, dur))
                    stutter_time += dur
                in_stutter = False
    if in_stutter:
        dur = results[-1][0] - stutter_start
        if dur >= 0.5:
            stutters.append((stutter_start, results[-1][0], dur))
            stutter_time += dur

    return {
        "auto_dur": auto_dur,
        "peak_setp": peak_setp,
        "avg_setp": avg_setp,
        "avg_err": avg_err,
        "peak_err": peak_err,
        "above_max": above_max,
        "above_max_total": len(setpoints),
        "big_err_count": big_err_count,
        "mod_above": mod_above,
        "mod_total": mod_total,
        "stutter_time": stutter_time,
        "stutters": stutters,
        "timeline": results,
    }


# ── Run all matches ─────────────────────────────────────────────
print(f"{'Match':>5s}  {'PathV':>5s}  {'AutoDur':>7s}  {'PeakSetp':>9s}  {'AvgSetp':>8s}  {'AvgErr':>7s}  {'PeakErr':>8s}  {'>6.02':>5s}  {'BigErr':>6s}  {'StutterT':>8s}  {'Verdict':>12s}")
print(f"{'─'*5}  {'─'*5}  {'─'*7}  {'─'*9}  {'─'*8}  {'─'*7}  {'─'*8}  {'─'*5}  {'─'*6}  {'─'*8}  {'─'*12}")

all_results = {}
for label, filename in LOGS:
    path = os.path.join(LOG_DIR, filename)
    r = analyze_match(label, path)
    if r is None:
        print(f"{label:>5s}  {PATH_VEL.get(label, '?'):>5.1f}  {'NO DATA':>7s}")
        continue

    all_results[label] = r
    pv = PATH_VEL.get(label, 0)

    verdict = ""
    if r["peak_setp"] > ROBOT_MAX:
        verdict = "⚠️ OVER MAX"
    elif r["stutter_time"] > 1.0:
        verdict = "⚠️ STUTTER"
    elif r["avg_err"] > 1.0:
        verdict = "⚠️ HIGH ERR"
    elif r["avg_err"] > 0.5:
        verdict = "~ marginal"
    else:
        verdict = "✅ OK"

    print(f"{label:>5s}  {pv:>5.1f}  {r['auto_dur']:>6.1f}s  {r['peak_setp']:>8.2f}m  {r['avg_setp']:>7.2f}m  {r['avg_err']:>6.2f}m  {r['peak_err']:>7.2f}m  {r['above_max']:>5d}  {r['big_err_count']:>6d}  {r['stutter_time']:>7.1f}s  {verdict:>12s}")

# ── Detail: show stutter periods for each match ────────────────
print(f"\n\n{'='*70}")
print("STUTTER PERIODS (continuous >1.0 m/s error for >0.5s)")
print(f"{'='*70}")

for label, filename in LOGS:
    if label not in all_results:
        continue
    r = all_results[label]
    if r["stutters"]:
        print(f"\n  {label} (path={PATH_VEL.get(label,0):.1f} m/s):")
        for s, e, d in r["stutters"]:
            print(f"    t={s:.1f}s – {e:.1f}s  ({d:.1f}s)")
    else:
        print(f"\n  {label}: No stutter periods detected")

# ── Velocity timeline for select matches ────────────────────────
print(f"\n\n{'='*70}")
print("VELOCITY TIMELINES (setpoint m/s → measured m/s, every 0.5s)")
print(f"{'='*70}")

for label in ["Q6", "Q12", "Q17", "Q23", "Q52", "Q61", "Q67"]:
    if label not in all_results:
        continue
    r = all_results[label]
    pv = PATH_VEL.get(label, 0)
    print(f"\n  {label} (path={pv:.1f} m/s):")
    print(f"    {'Time':>5s}  {'Setp':>6s}  {'Meas':>6s}  {'Err':>6s}")
    for t_rel, sp, ms, err in r["timeline"]:
        if t_rel % 0.5 < 0.01 or abs(t_rel % 0.5 - 0.5) < 0.01:  # every 0.5s
            flag = ""
            if sp > ROBOT_MAX:
                flag = " ⚠️>MAX"
            elif err > 2.0:
                flag = " ❌"
            elif err > 1.0:
                flag = " ⚠️"
            print(f"    {t_rel:>5.1f}  {sp:>6.2f}  {ms:>6.2f}  {err:>6.2f}{flag}")

print("\nDone.")
