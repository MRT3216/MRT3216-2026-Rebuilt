"""
MRT3216 — Boise Competition Log Analyzer
==========================================
Processes all wpilog files from the Boise (IDBO) competition.

For EACH match:
  • Match metadata (number, alliance, FMS)
  • Battery / power / energy
  • Loop timing & overrun statistics
  • Drive module currents
  • Shooter pipeline (flywheel, hood, kicker, spindexer)
  • Turret tracking & travel limits
  • Hybrid aiming performance
  • Hub shift timing
  • Vision pose estimation
  • Command activity
  • Alerts

CROSS-MATCH:
  • Comparative loop timing table
  • Loop overrun deep dive (clustering, correlation with battery/CAN/vision)
  • Trend analysis across the day
  • Actionable recommendations

Usage:
    .venv/Scripts/python.exe scripts/analyze_boise.py "C:/Users/danla/Desktop/Logs Idaho"
"""

import sys, os, glob, math, statistics, datetime
from collections import defaultdict
from wpiutil.log import DataLogReader

if len(sys.argv) < 2:
    print("Usage: python analyze_boise.py <log-directory>")
    sys.exit(1)

LOG_DIR = sys.argv[1]
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PATH = os.path.join(SCRIPT_DIR, "boise_analysis.txt")
_out = open(OUTPUT_PATH, "w", encoding="utf-8")

def p(*args, **kwargs):
    print(*args, **kwargs)
    print(*args, **kwargs, file=_out)
    _out.flush()

# ══════════════════════════════════════════════════════════════════════════════
# Parsing helpers
# ══════════════════════════════════════════════════════════════════════════════

def parse_wpilog(path):
    """Parse a wpilog into typed dictionaries."""
    entry_map = {}
    type_map = {}
    numeric = defaultdict(list)
    booleans = defaultdict(list)
    strings = defaultdict(list)
    string_arrays = defaultdict(list)
    doubles_arr = defaultdict(list)
    struct_counts = defaultdict(list)  # (t, element_count) for struct arrays
    # Only parse struct arrays for vision pose keys (performance)
    vision_struct_entries = set()

    for record in DataLogReader(path):
        if record.isStart():
            d = record.getStartData()
            entry_map[d.entry] = d.name
            type_map[d.entry] = d.type
            if ("Vision/Camera" in d.name and "Poses" in d.name
                    and d.type.startswith("struct:") and d.type.endswith("[]")):
                vision_struct_entries.add(d.entry)
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
            elif eid in vision_struct_entries:
                # Count Pose3d elements by raw byte length (56 bytes each)
                raw = record.getRaw()
                count = len(raw) // 56
                struct_counts[name].append((t, count))
        except:
            pass

    return numeric, booleans, strings, string_arrays, doubles_arr, struct_counts


def find_phases(en_pts, au_pts):
    """Find auto/teleop start and end times."""
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
        if not v and teleop_start and t > teleop_start:
            teleop_end = t
    return auto_start, auto_end, teleop_start, teleop_end


def stats(pairs):
    if not pairs:
        return None
    vals = [v for _, v in pairs]
    n = len(vals)
    if n == 0:
        return None
    avg = sum(vals) / n
    mn = min(vals)
    mx = max(vals)
    sv = sorted(vals)
    p50 = sv[n // 2]
    p95 = sv[int(n * 0.95)]
    p99 = sv[int(n * 0.99)]
    return {"n": n, "min": mn, "max": mx, "avg": avg, "p50": p50, "p95": p95, "p99": p99}


def win(data, t_start, t_end):
    return [(t, v) for t, v in data if t_start <= t <= t_end]


def bool_duty(pts, t_start, t_end, hz=50):
    if not pts:
        return None
    state = False
    for pt, pv in pts:
        if pt <= t_start: state = pv
    dt = 1.0 / hz
    total = active = 0
    t = t_start
    transitions = [(pt, pv) for pt, pv in pts if t_start < pt <= t_end]
    ti = 0
    while t <= t_end:
        while ti < len(transitions) and transitions[ti][0] <= t:
            state = transitions[ti][1]
            ti += 1
        total += 1
        if state: active += 1
        t += dt
    return active / total if total > 0 else 0


# ══════════════════════════════════════════════════════════════════════════════
# Per-match analysis
# ══════════════════════════════════════════════════════════════════════════════

def analyze_match(path, label):
    """Full per-match analysis. Returns a summary dict for cross-match comparison."""
    numeric, booleans, strings, string_arrays, doubles_arr, struct_counts = parse_wpilog(path)

    en_pts = booleans["/DriverStation/Enabled"]
    au_pts = booleans["/DriverStation/Autonomous"]
    AS, AE, TS, TE = find_phases(en_pts, au_pts)

    fsize = os.path.getsize(path) / 1e6
    log_dur = max(t for t, _ in en_pts) - min(t for t, _ in en_pts) if en_pts else 0

    p("\n" + "█" * 80)
    p(f"  MATCH: {label}")
    p(f"  File: {os.path.basename(path)}  ({fsize:.1f} MB)")
    p(f"  Duration: {log_dur:.0f}s")
    p("█" * 80)

    # Match info
    mn = numeric.get("/DriverStation/MatchNumber", [])
    mt = numeric.get("/DriverStation/MatchType", [])
    match_num = int(mn[-1][1]) if mn else 0
    if mn: p(f"  Match #: {match_num}")
    if mt:
        v = int(mt[-1][1])
        p(f"  Type: {['None','Practice','Qual','Elim'][v] if v < 4 else v}")
    fms = booleans.get("/DriverStation/FMSAttached", [])
    fms_on = any(v for _, v in fms)
    p(f"  FMS: {'Yes' if fms_on else 'No'}")
    alliance = numeric.get("/DriverStation/AllianceStation", [])
    if alliance:
        a = int(alliance[-1][1])
        labels = ["Red1","Red2","Red3","Blue1","Blue2","Blue3"]
        p(f"  Station: {labels[a] if a < 6 else a}")

    if AS and AE: p(f"  AUTO:   {AS:.1f}s – {AE:.1f}s  ({AE-AS:.1f}s)")
    if TS and TE: p(f"  TELEOP: {TS:.1f}s – {TE:.1f}s  ({TE-TS:.1f}s)")

    result = {"label": label, "path": path, "match_num": match_num}

    if not TS or not TE:
        p("  ⚠ No teleop phase found — skipping detailed analysis")
        return result

    # ── CRT ──
    p("\n  ── CRT Encoder ──")
    crt_s = strings.get("/RealOutputs/Shooter/Turret/CRT/Status", [])
    if crt_s:
        p(f"  Status: {crt_s[0][1]}")
        crt_err = numeric.get("/RealOutputs/Shooter/Turret/CRT/ErrorRot", [])
        crt_res = numeric.get("/RealOutputs/Shooter/Turret/CRT/ResolvedDeg", [])
        if crt_err: p(f"  Error: {crt_err[0][1]:.6f} rot")
        if crt_res: p(f"  Resolved: {crt_res[0][1]:.2f}°")

    # ── Battery / Power ──
    p("\n  ── Battery / Power (teleop) ──")
    bv = win(numeric.get("/SystemStats/BatteryVoltage", []), TS, TE)
    bc = win(numeric.get("/SystemStats/BatteryCurrent", []), TS, TE)
    bv_s = stats(bv)
    bc_s = stats(bc)
    if bv_s:
        p(f"  Voltage: avg={bv_s['avg']:.2f}V  min={bv_s['min']:.2f}V  max={bv_s['max']:.2f}V")
        result["batt_min"] = bv_s["min"]
        result["batt_avg"] = bv_s["avg"]
    if bc_s:
        p(f"  Current: avg={bc_s['avg']:.1f}A  p95={bc_s['p95']:.1f}A  max={bc_s['max']:.1f}A")

    bo = bool_duty(booleans.get("/SystemStats/BrownedOut", []), TS, TE)
    if bo is not None:
        if bo > 0: p(f"  *** BROWNOUT: {bo*100:.1f}% of teleop ***")
        else: p("  No brownouts")

    if bv:
        below10 = sum(1 for _, v in bv if v < 10.0)
        if below10: p(f"  *** {below10} samples below 10V ***")

    te_energy = numeric.get("/RealOutputs/EnergyLogger/TotalEnergyJoules", [])
    if te_energy:
        e_start = [v for t, v in te_energy if t <= TS]
        e_end = [v for t, v in te_energy if t <= TE]
        if e_start and e_end:
            delta = e_end[-1] - e_start[-1]
            p(f"  Energy (teleop): {delta:.0f} J  ({delta/3600:.1f} Wh)")
            result["energy_j"] = delta

    can_util = win(numeric.get("/SystemStats/CANBus/Utilization", []), TS, TE)
    can_s = stats(can_util)
    if can_s:
        p(f"  CAN util: avg={can_s['avg']*100:.1f}%  p95={can_s['p95']*100:.1f}%  max={can_s['max']*100:.1f}%")
        result["can_max"] = can_s["max"]

    cpu = win(numeric.get("/SystemStats/CPUTempCelsius", []), TS, TE)
    cpu_s = stats(cpu)
    if cpu_s:
        p(f"  CPU temp: avg={cpu_s['avg']:.1f}°C  max={cpu_s['max']:.1f}°C")

    # ── Loop Timing ──
    p("\n  ── Loop Timing (enabled) ──")
    loop_data = []
    user_data = []
    gc_data = []
    if AS and AE:
        loop_data.extend(win(numeric.get("/RealOutputs/LoggedRobot/FullCycleMS", []), AS, AE))
        user_data.extend(win(numeric.get("/RealOutputs/LoggedRobot/UserCodeMS", []), AS, AE))
        gc_data.extend(win(numeric.get("/RealOutputs/LoggedRobot/GCTimeMS", []), AS, AE))
    loop_data.extend(win(numeric.get("/RealOutputs/LoggedRobot/FullCycleMS", []), TS, TE))
    user_data.extend(win(numeric.get("/RealOutputs/LoggedRobot/UserCodeMS", []), TS, TE))
    gc_data.extend(win(numeric.get("/RealOutputs/LoggedRobot/GCTimeMS", []), TS, TE))

    loop_s = stats(loop_data)
    user_s = stats(user_data)
    gc_s = stats(gc_data)

    if loop_s:
        over20 = sum(1 for _, v in loop_data if v > 20)
        over30 = sum(1 for _, v in loop_data if v > 30)
        over50 = sum(1 for _, v in loop_data if v > 50)
        p(f"  Full cycle: avg={loop_s['avg']:.2f}ms  p95={loop_s['p95']:.2f}ms  p99={loop_s['p99']:.2f}ms  max={loop_s['max']:.2f}ms")
        p(f"  Overruns:  >20ms={over20} ({100*over20/loop_s['n']:.1f}%)  >30ms={over30}  >50ms={over50}")
        result["loop_avg"] = loop_s["avg"]
        result["loop_p95"] = loop_s["p95"]
        result["loop_p99"] = loop_s["p99"]
        result["loop_max"] = loop_s["max"]
        result["loop_n"] = loop_s["n"]
        result["over20"] = over20
        result["over20_pct"] = 100 * over20 / loop_s["n"]
        result["over30"] = over30
        result["over50"] = over50
        result["loop_data"] = loop_data  # for cross-match analysis
    if user_s:
        p(f"  User code: avg={user_s['avg']:.2f}ms  p95={user_s['p95']:.2f}ms  p99={user_s['p99']:.2f}ms  max={user_s['max']:.2f}ms")
        result["user_avg"] = user_s["avg"]
        result["user_p95"] = user_s["p95"]
    if gc_s:
        p(f"  GC time:   avg={gc_s['avg']:.2f}ms  p95={gc_s['p95']:.2f}ms  max={gc_s['max']:.2f}ms")
        result["gc_avg"] = gc_s["avg"]
        result["gc_p95"] = gc_s["p95"]
        result["gc_max"] = gc_s["max"]

    # AKit logger breakdown
    queued = win(numeric.get("/RealOutputs/Logger/QueuedCycles", []), TS, TE)
    q_s = stats(queued)
    if q_s:
        p(f"  Logger queue: avg={q_s['avg']:.1f}  max={int(q_s['max'])}")

    autolog = win(numeric.get("/RealOutputs/Logger/AutoLogMS", []), TS, TE)
    conduit_cap = win(numeric.get("/RealOutputs/Logger/ConduitCaptureMS", []), TS, TE)
    conduit_save = win(numeric.get("/RealOutputs/Logger/ConduitSaveMS", []), TS, TE)
    for key_name, data_pts in [("AutoLogMS", autolog), ("ConduitCapture", conduit_cap), ("ConduitSave", conduit_save)]:
        s = stats(data_pts)
        if s and s["max"] > 0.5:
            p(f"  Logger/{key_name}: avg={s['avg']:.2f}ms  p95={s['p95']:.2f}ms  max={s['max']:.2f}ms")

    # Worst 5 loops
    if loop_data:
        worst5 = sorted(loop_data, key=lambda x: -x[1])[:5]
        p("  5 worst cycles:")
        for t, v in worst5:
            phase = "AUTO" if AS and AS <= t <= AE else "TELE" if TS <= t <= TE else "DIS"
            # Check GC at that time
            gc_note = ""
            for gt, gv in gc_data:
                if abs(gt - t) < 0.025:
                    if gv > 2:
                        gc_note = f"  GC={gv:.1f}ms"
                    break
            # Check battery
            batt_note = ""
            for bt, bvv in bv:
                if abs(bt - t) < 0.1:
                    batt_note = f"  batt={bvv:.1f}V"
                    break
            p(f"    t={t:.2f}s [{phase}]  {v:.1f}ms{gc_note}{batt_note}")

    # ── Loop Overrun Deep Dive ──
    p("\n  ── Loop Overrun Deep Dive ──")
    if loop_data and loop_s:
        # Phase breakdown
        tele_loops = win(loop_data, TS, TE)
        auto_loops = win(loop_data, AS, AE) if AS and AE else []
        tele_vals = [v for _, v in tele_loops]
        auto_vals = [v for _, v in auto_loops]
        if tele_vals:
            tele_over = sum(1 for v in tele_vals if v > 20)
            p(f"  Teleop: {len(tele_vals)} loops, {tele_over} overruns ({100*tele_over/len(tele_vals):.1f}%)")
        if auto_vals:
            auto_over = sum(1 for v in auto_vals if v > 20)
            p(f"  Auto:   {len(auto_vals)} loops, {auto_over} overruns ({100*auto_over/len(auto_vals):.1f}%)")

        # Distribution histogram
        buckets = [0, 10, 15, 20, 25, 30, 40, 50, 75, 100, 200]
        all_vals = [v for _, v in loop_data]
        p("  Distribution:")
        for i in range(len(buckets) - 1):
            cnt = sum(1 for v in all_vals if buckets[i] <= v < buckets[i+1])
            if cnt > 0:
                bar = "█" * min(cnt * 50 // max(len(all_vals), 1), 50)
                p(f"    {buckets[i]:>4d}-{buckets[i+1]:>4d}ms: {cnt:>5d} ({100*cnt/len(all_vals):>5.1f}%) {bar}")
        over = sum(1 for v in all_vals if v >= buckets[-1])
        if over > 0:
            p(f"    {buckets[-1]:>4d}+   ms: {over:>5d} ({100*over/len(all_vals):>5.1f}%)")

        # Overrun bursts (>=3 consecutive >20ms)
        bursts = []
        burst_start = None
        burst_count = 0
        burst_vals = []
        for t, v in loop_data:
            if v > 20:
                if burst_start is None:
                    burst_start = t
                burst_count += 1
                burst_vals.append(v)
            else:
                if burst_count >= 3:
                    bursts.append((burst_start, t, burst_count, max(burst_vals), sum(burst_vals)/len(burst_vals)))
                burst_start = None
                burst_count = 0
                burst_vals = []
        if burst_count >= 3:
            bursts.append((burst_start, loop_data[-1][0], burst_count, max(burst_vals), sum(burst_vals)/len(burst_vals)))

        if bursts:
            p(f"  Overrun bursts (≥3 consecutive >20ms): {len(bursts)} total")
            for bs, be, bc, bmax, bavg in bursts[:8]:
                phase = "AUTO" if AS and AS <= bs <= AE else "TELE" if TS and TS <= bs <= TE else "DIS"
                p(f"    t={bs:.1f}–{be:.1f}s [{phase}] {bc} cycles, avg={bavg:.1f}ms, peak={bmax:.1f}ms")
            if len(bursts) > 8:
                p(f"    ... {len(bursts)} total bursts")
        result["bursts"] = len(bursts)

        # GC correlation
        if gc_data:
            slow_loops_with_gc = 0
            slow_loops_total = 0
            for lt, lv in loop_data:
                if lv > 20:
                    slow_loops_total += 1
                    for gt, gv in gc_data:
                        if abs(gt - lt) < 0.025 and gv > 2:
                            slow_loops_with_gc += 1
                            break
            if slow_loops_total > 0:
                p(f"  GC correlation: {slow_loops_with_gc}/{slow_loops_total} overruns ({100*slow_loops_with_gc/slow_loops_total:.0f}%) had GC > 2ms")
                result["gc_corr_pct"] = 100 * slow_loops_with_gc / slow_loops_total

        # Battery sag correlation
        if bv:
            slow_batt = []
            fast_batt = []
            for lt, lv in loop_data:
                for bt, bvv in bv:
                    if abs(bt - lt) < 0.1:
                        if lv > 20:
                            slow_batt.append(bvv)
                        else:
                            fast_batt.append(bvv)
                        break
            if slow_batt and fast_batt:
                p(f"  Battery during overruns: avg={statistics.mean(slow_batt):.2f}V  (normal: {statistics.mean(fast_batt):.2f}V)")

    # ── Drive ──
    p("\n  ── Drive (teleop) ──")
    total_drive_current = 0
    for i in range(4):
        dc = win(numeric.get(f"/Drive/Module{i}/DriveCurrentAmps", []), TS, TE)
        tc = win(numeric.get(f"/Drive/Module{i}/TurnCurrentAmps", []), TS, TE)
        dc_s = stats(dc)
        tc_s = stats(tc)
        if dc_s:
            p(f"  Mod{i} Drive: avg={dc_s['avg']:.1f}A  p95={dc_s['p95']:.1f}A  max={dc_s['max']:.1f}A")
            total_drive_current += dc_s["avg"]
        if tc_s:
            p(f"  Mod{i} Turn:  avg={tc_s['avg']:.1f}A  p95={tc_s['p95']:.1f}A  max={tc_s['max']:.1f}A")
            total_drive_current += tc_s["avg"]
    result["drive_avg_current"] = total_drive_current

    # ── Shooter ──
    p("\n  ── Shooter Pipeline (teleop) ──")

    # Flywheel
    fw_vel = win(numeric.get("/RealOutputs/Flywheel/FX/VelocityRPM", []), TS, TE)
    fw_ref = win(numeric.get("/RealOutputs/Flywheel/FX/ReferenceRPM", []), TS, TE)
    fw_cur = win(numeric.get("/Shooter/Flywheel/Current", []), TS, TE)

    p("  Flywheel:")
    if fw_ref:
        commanded = [(t, v) for t, v in fw_ref if abs(v) > 100]
        p(f"    Samples: {len(fw_vel)} total, {len(commanded)} commanded")
        if commanded:
            cs = stats(commanded)
            p(f"    Ref (active): avg={cs['avg']:.0f}  min={cs['min']:.0f}  max={cs['max']:.0f} RPM")
    if fw_vel and fw_ref and len(fw_vel) == len(fw_ref):
        errors = [(t, abs(vel - ref)) for (t, vel), (_, ref) in zip(fw_vel, fw_ref) if abs(ref) > 100]
        if errors:
            es = stats(errors)
            p(f"    Tracking error: avg={es['avg']:.0f}  p95={es['p95']:.0f}  max={es['max']:.0f} RPM")
    spun_duty = bool_duty(booleans.get("/RealOutputs/Flywheel/IsSpunUp", []), TS, TE)
    if spun_duty is not None:
        p(f"    Spun-up: {spun_duty*100:.1f}% of teleop")
    if fw_cur:
        fc_s = stats(fw_cur)
        p(f"    Current: avg={fc_s['avg']:.1f}A  p95={fc_s['p95']:.1f}A  max={fc_s['max']:.1f}A")

    # Hood
    p("  Hood:")
    hood_pos = win(numeric.get("/RealOutputs/Hood/FX/PositionDegrees", []), TS, TE)
    hood_ref = win(numeric.get("/RealOutputs/Hood/FX/ReferenceDegrees", []), TS, TE)
    if hood_pos:
        hs = stats(hood_pos)
        p(f"    Position: avg={hs['avg']:.1f}°  min={hs['min']:.1f}°  max={hs['max']:.1f}°")
    if hood_pos and hood_ref and len(hood_pos) == len(hood_ref):
        herr = [(t, abs(pp - r)) for (t, pp), (_, r) in zip(hood_pos, hood_ref)]
        hes = stats(herr)
        if hes:
            p(f"    Tracking error: avg={hes['avg']:.2f}°  p95={hes['p95']:.2f}°  max={hes['max']:.2f}°")

    # Kicker / Spindexer
    k_cur = win(numeric.get("/Kicker/Current", []), TS, TE)
    sp_cur = win(numeric.get("/Spindexer/Current", []), TS, TE)
    if k_cur:
        kcs = stats(k_cur)
        p(f"  Kicker current: avg={kcs['avg']:.1f}A  max={kcs['max']:.1f}A")
    if sp_cur:
        scs = stats(sp_cur)
        p(f"  Spindexer current: avg={scs['avg']:.1f}A  max={scs['max']:.1f}A")

    # ── Turret ──
    p("\n  ── Turret (teleop) ──")
    turret_pos = win(numeric.get("/RealOutputs/Shooter/Turret/PositionDegrees", []), TS, TE)
    turret_cur = win(numeric.get("/Shooter/Turret/Current", []), TS, TE)
    if turret_pos:
        tp_s = stats(turret_pos)
        p(f"  Position: avg={tp_s['avg']:.1f}°  min={tp_s['min']:.1f}°  max={tp_s['max']:.1f}°")
        near_min = sum(1 for _, v in turret_pos if v < -85)
        near_max = sum(1 for _, v in turret_pos if v > 125)
        if near_min or near_max:
            p(f"  Near limits: {near_min} @ min, {near_max} @ max  ({100*(near_min+near_max)/len(turret_pos):.1f}%)")
        result["turret_near_limits_pct"] = 100 * (near_min + near_max) / len(turret_pos) if turret_pos else 0
    if turret_cur:
        tc_s = stats(turret_cur)
        p(f"  Current: avg={tc_s['avg']:.1f}A  p95={tc_s['p95']:.1f}A  max={tc_s['max']:.1f}A")

    # ── Hybrid Aiming ──
    p("\n  ── Hybrid Aiming (teleop) ──")
    aim_duty = bool_duty(booleans.get("/RealOutputs/HybridAiming/aimEnabled", []), TS, TE)
    if aim_duty is not None:
        p(f"  Aim enabled: {aim_duty*100:.1f}% of teleop")
    clamped_duty = bool_duty(booleans.get("/RealOutputs/HybridAiming/turretClamped", []), TS, TE)
    if clamped_duty is not None:
        p(f"  Turret clamped: {clamped_duty*100:.1f}% of teleop")
        result["clamped_pct"] = clamped_duty * 100
    outside_duty = bool_duty(booleans.get("/RealOutputs/HybridAiming/outsideTravelWindow", []), TS, TE)
    if outside_duty is not None:
        p(f"  Outside travel window: {outside_duty*100:.1f}% of teleop")
    raw_az = win(numeric.get("/RealOutputs/HybridAiming/rawTurretAzimuthDeg", []), TS, TE)
    if raw_az:
        ra_s = stats(raw_az)
        p(f"  Raw azimuth: min={ra_s['min']:.1f}°  avg={ra_s['avg']:.1f}°  max={ra_s['max']:.1f}°")

    # ── Hub Shift ──
    p("\n  ── Hub Shift (teleop) ──")
    shifted_duty = bool_duty(booleans.get("/RealOutputs/HubShift/ShiftedActive", []), TS, TE)
    if shifted_duty is not None:
        p(f"  Shifted-active: {shifted_duty*100:.1f}% of teleop")
    shift_strs = [(t, s) for t, s in strings.get("/RealOutputs/HubShift/CurrentShift", []) if TS <= t <= TE]
    if shift_strs:
        prev = None
        p("  Shift transitions:")
        for t, s in shift_strs:
            if s != prev:
                p(f"    T+{t-TS:.0f}s → {s}")
                prev = s

    # ── Shooter Telemetry ──
    p("\n  ── Shooter Telemetry (teleop) ──")
    hub_dist = win(numeric.get("/RealOutputs/ShooterTelemetry/hubDistanceMeters", []), TS, TE)
    valid_duty = bool_duty(booleans.get("/RealOutputs/ShooterTelemetry/isValid", []), TS, TE)
    if valid_duty is not None:
        p(f"  Solution valid: {valid_duty*100:.1f}% of teleop")
        result["solution_valid_pct"] = valid_duty * 100
    if hub_dist:
        active_dist = [(t, v) for t, v in hub_dist if v > 0.1]
        if active_dist:
            hd_s = stats(active_dist)
            p(f"  Hub distance: avg={hd_s['avg']:.2f}m  min={hd_s['min']:.2f}m  max={hd_s['max']:.2f}m")
    rpm_fudge = win(numeric.get("/RealOutputs/ShooterTelemetry/rpmFudgeRPM", []), TS, TE)
    if rpm_fudge:
        p(f"  RPM fudge: final={rpm_fudge[-1][1]:.0f}")

    # ── Vision ──
    p("\n  ── Vision (teleop) ──")

    # Summary-level keys (easy to parse, reliable)
    tag_count = win(numeric.get("/RealOutputs/Vision/Summary/TagCount", []), TS, TE)
    has_target = booleans.get("/RealOutputs/Vision/Summary/HasTarget", [])
    target_duty = bool_duty(has_target, TS, TE)
    if tag_count:
        tc_s = stats(tag_count)
        nonzero = sum(1 for _, v in tag_count if v > 0)
        p(f"  Tag count: avg={tc_s['avg']:.1f}  max={int(tc_s['max'])}  "
          f"frames w/tags: {nonzero}/{len(tag_count)} ({100*nonzero/len(tag_count):.0f}%)")
        result["vision_tag_pct"] = 100 * nonzero / len(tag_count)
    if target_duty is not None:
        p(f"  Has target: {target_duty*100:.1f}% of teleop")

    # Per-camera connected status & pose counts from struct arrays
    total_accepted_all = 0
    total_rejected_all = 0
    total_poses_all = 0
    for cam in range(4):
        # Connected status (AKit processInputs: /Vision/CameraX/Connected)
        conn_pts = booleans.get(f"/Vision/Camera{cam}/Connected", [])
        conn_state = None
        for ct, cv in conn_pts:
            if ct <= TE: conn_state = cv
        conn_str = "Yes" if conn_state else ("No" if conn_state is False else "?")

        # Struct array counts for poses
        poses = win(struct_counts.get(f"/RealOutputs/Vision/Camera{cam}/RobotPoses", []), TS, TE)
        acc = win(struct_counts.get(f"/RealOutputs/Vision/Camera{cam}/RobotPosesAccepted", []), TS, TE)
        rej = win(struct_counts.get(f"/RealOutputs/Vision/Camera{cam}/RobotPosesRejected", []), TS, TE)

        tot_poses = sum(v for _, v in poses)
        tot_acc = sum(v for _, v in acc)
        tot_rej = sum(v for _, v in rej)
        total_poses_all += tot_poses
        total_accepted_all += tot_acc
        total_rejected_all += tot_rej

        if poses:
            pct = 100 * tot_acc / tot_poses if tot_poses > 0 else 0
            p(f"  Cam{cam} [{conn_str}]: {tot_poses} poses, {tot_acc} accepted ({pct:.0f}%), {tot_rej} rejected")
        else:
            p(f"  Cam{cam} [{conn_str}]: no pose data")

    if total_poses_all > 0:
        accept_rate = 100 * total_accepted_all / total_poses_all
        p(f"  TOTAL: {total_poses_all} poses, {total_accepted_all} accepted ({accept_rate:.0f}%), {total_rejected_all} rejected")
        result["vision_accept_rate"] = accept_rate
        result["vision_total_poses"] = total_poses_all
        result["vision_accepted"] = total_accepted_all

    # ── Commands ──
    p("\n  ── Command Activity (teleop) ──")
    cmd_prefix = "/RealOutputs/CommandsAll/"
    cmd_names = [n for n in booleans if n.startswith(cmd_prefix)]
    cmd_duties = []
    for name in sorted(cmd_names):
        duty = bool_duty(booleans[name], TS, TE)
        if duty is not None and duty > 0.001:
            label_name = name[len(cmd_prefix):]
            active_s = duty * (TE - TS)
            cmd_duties.append((duty, active_s, label_name))
    cmd_duties.sort(key=lambda x: -x[0])
    for duty, active_s, lbl in cmd_duties:
        p(f"    {lbl:<45} {active_s:>5.1f}s  ({duty*100:>5.1f}%)")

    # ── Alerts ──
    p("\n  ── Alerts ──")
    all_errors = set()
    all_warnings = set()
    for t, arr in string_arrays.get("/RealOutputs/Alerts/errors", []):
        for e in arr:
            if e: all_errors.add(e)
    for t, arr in string_arrays.get("/RealOutputs/Alerts/warnings", []):
        for w in arr:
            if w: all_warnings.add(w)
    for e in sorted(all_errors): p(f"    ❌ {e}")
    for w in sorted(all_warnings): p(f"    ⚠️  {w}")
    if not all_errors and not all_warnings: p("    No alerts")

    return result


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    p("=" * 80)
    p("  MRT3216 BOISE (IDBO) COMPETITION LOG ANALYSIS")
    p(f"  Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    p(f"  Log directory: {LOG_DIR}")
    p("=" * 80)

    # Find all wpilog files
    logs = sorted(glob.glob(os.path.join(LOG_DIR, "*.wpilog")))
    p(f"\n  Found {len(logs)} wpilog files")
    for l in logs:
        p(f"    {os.path.basename(l)}  ({os.path.getsize(l)/1e6:.1f} MB)")

    if not logs:
        p("  No wpilog files found!")
        _out.close()
        return

    # ── Per-match analysis ──
    results = []
    for log_path in logs:
        basename = os.path.basename(log_path)
        # Extract match label from filename
        # e.g. akit_26-03-27_17-41-24_idbo_q6.wpilog → IDBO Q6
        label = basename.replace("akit_", "").replace(".wpilog", "")
        parts = basename.lower().replace(".wpilog", "").split("_")
        event = ""
        match_id = ""
        for part in parts:
            if part.isalpha() and part not in ("akit",):
                event = part.upper()
            elif part[0] == "q" and part[1:].isdigit():
                match_id = f"Q{part[1:]}"
        if event and match_id:
            label = f"{event} {match_id}"
        elif match_id:
            label = match_id

        try:
            result = analyze_match(log_path, label)
            results.append(result)
        except Exception as e:
            p(f"\n  ⚠ Error analyzing {basename}: {e}")
            import traceback
            traceback.print_exc()

    if not results:
        p("\n  No matches analyzed successfully!")
        _out.close()
        return

    # ══════════════════════════════════════════════════════════════════════════
    # CROSS-MATCH COMPARATIVE ANALYSIS
    # ══════════════════════════════════════════════════════════════════════════
    p("\n\n" + "█" * 80)
    p("  CROSS-MATCH COMPARATIVE ANALYSIS")
    p("█" * 80)

    # ── Loop Timing Comparison Table ──
    p("\n  ── Loop Timing Across All Matches ──")
    p(f"  {'Match':<15s}  {'Avg':>6s}  {'p95':>6s}  {'p99':>6s}  {'Max':>7s}  {'>20ms':>8s}  {'>30ms':>6s}  {'>50ms':>6s}  {'Bursts':>6s}")
    p(f"  {'-'*15}  {'---':>6s}  {'---':>6s}  {'---':>6s}  {'---':>7s}  {'---':>8s}  {'---':>6s}  {'---':>6s}  {'---':>6s}")
    for r in results:
        if "loop_avg" not in r:
            continue
        pct = f"{r['over20_pct']:.1f}%"
        p(f"  {r['label']:<15s}  {r['loop_avg']:>5.1f}ms  {r['loop_p95']:>5.1f}ms  {r['loop_p99']:>5.1f}ms  "
          f"{r['loop_max']:>6.1f}ms  {r['over20']:>5d} {pct:>5s}  {r['over30']:>6d}  {r['over50']:>6d}  "
          f"{r.get('bursts', 0):>6d}")

    # ── Battery Comparison ──
    p("\n  ── Battery Across All Matches ──")
    for r in results:
        if "batt_avg" in r:
            p(f"  {r['label']:<15s}  avg={r['batt_avg']:.2f}V  min={r['batt_min']:.2f}V  "
              f"energy={r.get('energy_j', 0):.0f}J")

    # ── GC Correlation Summary ──
    p("\n  ── GC Correlation Summary ──")
    for r in results:
        gc_pct = r.get("gc_corr_pct", None)
        if gc_pct is not None:
            gc_avg = r.get("gc_avg", 0)
            gc_p95 = r.get("gc_p95", 0)
            gc_max = r.get("gc_max", 0)
            p(f"  {r['label']:<15s}  {gc_pct:.0f}% overruns had GC>2ms  "
              f"GC: avg={gc_avg:.2f}ms p95={gc_p95:.2f}ms max={gc_max:.2f}ms")

    # ── Turret & Aiming Summary ──
    p("\n  ── Turret / Aiming Summary ──")
    for r in results:
        clamped = r.get("clamped_pct", None)
        limits = r.get("turret_near_limits_pct", None)
        valid = r.get("solution_valid_pct", None)
        parts = [r["label"]]
        if clamped is not None: parts.append(f"clamped={clamped:.1f}%")
        if limits is not None: parts.append(f"near-limits={limits:.1f}%")
        if valid is not None: parts.append(f"solution-valid={valid:.1f}%")
        if len(parts) > 1:
            p(f"  {'  '.join(parts)}")

    # ── Vision Summary ──
    p("\n  ── Vision Across All Matches ──")
    p(f"  {'Match':<15s}  {'Poses':>7s}  {'Accepted':>9s}  {'Rate':>6s}  {'Tag%':>5s}")
    p(f"  {'-'*15}  {'---':>7s}  {'---':>9s}  {'---':>6s}  {'---':>5s}")
    for r in results:
        poses = r.get("vision_total_poses", 0)
        acc = r.get("vision_accepted", 0)
        rate = r.get("vision_accept_rate", 0)
        tag_pct = r.get("vision_tag_pct", 0)
        p(f"  {r['label']:<15s}  {poses:>7d}  {acc:>9d}  {rate:>5.0f}%  {tag_pct:>4.0f}%")

    # ══════════════════════════════════════════════════════════════════════════
    # LOOP OVERRUN DEEP DIVE
    # ══════════════════════════════════════════════════════════════════════════
    p("\n\n" + "█" * 80)
    p("  LOOP OVERRUN DEEP INVESTIGATION")
    p("█" * 80)

    # Aggregate all loop data
    all_loop_vals = []
    for r in results:
        if "loop_data" in r:
            all_loop_vals.extend([v for _, v in r["loop_data"]])

    if all_loop_vals:
        total_cycles = len(all_loop_vals)
        total_over20 = sum(1 for v in all_loop_vals if v > 20)
        p(f"\n  Total cycles across all matches: {total_cycles:,}")
        p(f"  Total overruns (>20ms): {total_over20:,} ({100*total_over20/total_cycles:.1f}%)")
        p(f"  Aggregate mean: {statistics.mean(all_loop_vals):.2f}ms")
        p(f"  Aggregate p95:  {sorted(all_loop_vals)[int(0.95*len(all_loop_vals))]:.2f}ms")
        p(f"  Aggregate p99:  {sorted(all_loop_vals)[int(0.99*len(all_loop_vals))]:.2f}ms")
        p(f"  Aggregate max:  {max(all_loop_vals):.2f}ms")

        # Global distribution
        p("\n  Global distribution:")
        buckets = [0, 10, 15, 20, 25, 30, 40, 50, 75, 100, 200]
        for i in range(len(buckets) - 1):
            cnt = sum(1 for v in all_loop_vals if buckets[i] <= v < buckets[i+1])
            if cnt > 0:
                bar = "█" * min(cnt * 50 // max(total_cycles, 1), 50)
                p(f"    {buckets[i]:>4d}-{buckets[i+1]:>4d}ms: {cnt:>6d} ({100*cnt/total_cycles:>5.1f}%) {bar}")
        over = sum(1 for v in all_loop_vals if v >= buckets[-1])
        if over > 0:
            p(f"    {buckets[-1]:>4d}+   ms: {over:>6d} ({100*over/total_cycles:>5.1f}%)")

    # ── Trend across the day ──
    p("\n  ── Trend Across Matches (chronological) ──")
    p("  Are overruns getting worse as the day progresses?")
    for i, r in enumerate(results):
        if "loop_avg" in r:
            trend = "↑" if i > 0 and r["over20_pct"] > results[i-1].get("over20_pct", 0) else "↓" if i > 0 and r["over20_pct"] < results[i-1].get("over20_pct", 0) else "─"
            p(f"    {r['label']:<15s}  overrun={r['over20_pct']:>5.1f}%  avg={r['loop_avg']:>5.1f}ms  "
              f"user={r.get('user_avg', 0):>5.1f}ms  {trend}")

    # ── Root Cause Analysis ──
    p("\n  ── Root Cause Analysis ──")
    gc_corr_vals = [r.get("gc_corr_pct", 0) for r in results if "gc_corr_pct" in r]
    user_avgs = [r.get("user_avg", 0) for r in results if "user_avg" in r]
    loop_avgs = [r.get("loop_avg", 0) for r in results if "loop_avg" in r]

    if gc_corr_vals:
        avg_gc_corr = statistics.mean(gc_corr_vals)
        p(f"  GC contribution: {avg_gc_corr:.0f}% of overruns correlate with GC > 2ms")
        if avg_gc_corr > 50:
            p("    → GC is a MAJOR contributor. Consider: -XX:+UseG1GC, reduce allocations, object pooling.")
        elif avg_gc_corr > 20:
            p("    → GC is a MODERATE contributor. Audit hot-path allocations.")
        else:
            p("    → GC is not the primary cause.")

    if user_avgs and loop_avgs:
        overhead = statistics.mean(loop_avgs) - statistics.mean(user_avgs)
        p(f"  AKit/logging overhead: ~{overhead:.1f}ms per cycle (loop_avg - user_avg)")
        if overhead > 5:
            p("    → Significant logging overhead. Consider reducing telemetry verbosity or signal count.")
        elif overhead > 2:
            p("    → Moderate logging overhead.")

    # ══════════════════════════════════════════════════════════════════════════
    # RECOMMENDATIONS
    # ══════════════════════════════════════════════════════════════════════════
    p("\n\n" + "█" * 80)
    p("  RECOMMENDATIONS")
    p("█" * 80)

    recs = []

    # Loop timing
    if all_loop_vals and total_over20 / total_cycles > 0.05:
        recs.append(f"LOOP TIMING: {100*total_over20/total_cycles:.0f}% overrun rate across all matches. Target < 5%.")

    if gc_corr_vals and statistics.mean(gc_corr_vals) > 30:
        recs.append("GC PRESSURE: High GC correlation with overruns. Audit periodic() for allocations "
                     "(Rotation2d, Pose2d, Optional, lambda captures). Consider pre-allocating reusable objects.")

    # Battery
    batt_mins = [r.get("batt_min", 13) for r in results if "batt_min" in r]
    if batt_mins and min(batt_mins) < 9.0:
        recs.append(f"BATTERY: Voltage sagged to {min(batt_mins):.1f}V. Use fresher batteries or reduce current draw.")

    # CAN
    can_maxes = [r.get("can_max", 0) for r in results if "can_max" in r]
    if can_maxes and max(can_maxes) > 0.7:
        recs.append(f"CAN BUS: Utilization peaked at {max(can_maxes)*100:.0f}%. Reduce signal update rates for non-critical telemetry.")

    # Turret
    turret_limits = [r.get("turret_near_limits_pct", 0) for r in results if "turret_near_limits_pct" in r]
    if turret_limits and statistics.mean(turret_limits) > 5:
        recs.append(f"TURRET LIMITS: Turret near limits avg {statistics.mean(turret_limits):.0f}% of time. "
                     "Consider wider travel or driver strategy adjustment.")

    for i, rec in enumerate(recs, 1):
        p(f"  {i}. {rec}")
    if not recs:
        p("  No critical recommendations.")

    p(f"\n  Output saved to: {OUTPUT_PATH}")
    p("=" * 80)
    _out.close()
    print(f"\nDone! Report saved to: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
