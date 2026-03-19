import os
import re
import bisect
import sys
from collections import defaultdict
from wpiutil.log import DataLogReader

# Write output to file and stdout
output_path = r"C:\Users\danla\Documents\GitHub\MRT3216-2026-Rebuilt\scripts\match_output.txt"
_out = open(output_path, "w")

def p(*args, **kwargs):
    print(*args, **kwargs)
    print(*args, **kwargs, file=_out)
    _out.flush()

LOG_DIRS = [
    r"C:\Users\danla\Desktop\Logs",
    r"C:\Users\danla\Desktop\Logs2",
]

# Signals to pull and which subsystem they belong to
CURRENT_SIGNALS = {
    "/Drive/Module0/DriveCurrentAmps": "Drive",
    "/Drive/Module0/TurnCurrentAmps": "Drive",
    "/Drive/Module1/DriveCurrentAmps": "Drive",
    "/Drive/Module1/TurnCurrentAmps": "Drive",
    "/Drive/Module2/DriveCurrentAmps": "Drive",
    "/Drive/Module2/TurnCurrentAmps": "Drive",
    "/Drive/Module3/DriveCurrentAmps": "Drive",
    "/Drive/Module3/TurnCurrentAmps": "Drive",
    "/Intake/Pivot/Current": "IntakePivot",
    "/Intake/Rollers/Current": "IntakeRollers",
    "/Shooter/Turret/Current": "Turret",
    "/Shooter/Flywheel/Current": "Flywheel",
    "/Hood/Current": "Hood",
    "/Kicker/Current": "Kicker",
    "/Spindexer/Current": "Spindexer",
    "/SystemStats/BatteryVoltage": "_Battery",
    "/SystemStats/BatteryCurrent": "_BatteryCurrent",
}

TELEOP_SIGNAL = "/DriverStation/Enabled"
AUTON_SIGNAL = "/DriverStation/Autonomous"

# Downsample high-frequency signals to at most this many samples per match
# (keeps per-file parse under a second even for 130MB files)
MAX_RAW_SAMPLES_PER_SIGNAL = 6000


def parse_log(path):
    """
    Single-pass log parser.
    Returns a dict with all needed stats pre-computed so we never re-read the file.

    Keys returned:
      "teleop_data"    : signal -> list[float]  (values during teleop only, downsampled)
      "teleop_raw_bv"  : list[(t, v)]  for BatteryVoltage during teleop (all samples for min)
      "pivot_raw"      : list[(t, v)]  for IntakePivot during teleop (all samples for peak)
    """
    entry_map = {}   # entry_id -> signal name
    type_map  = {}   # entry_id -> type string

    # Accumulators — store every sample during the file scan (we filter to teleop after)
    raw = defaultdict(list)
    # For voltage and pivot we need per-sample timestamps too (min/peak detection)
    # but collecting all samples uses less RAM than re-reading, so we keep them compact:
    # store as flat parallel lists to avoid tuple overhead on 500k-sample files
    raw_t = defaultdict(list)  # entry_id -> timestamps (seconds)

    record_count = 0
    for record in DataLogReader(path):
        if record.isStart():
            d = record.getStartData()
            if d.name in CURRENT_SIGNALS or d.name in (TELEOP_SIGNAL, AUTON_SIGNAL):
                entry_map[d.entry] = d.name
                type_map[d.entry]  = d.type
            continue

        eid = record.getEntry()
        if eid not in entry_map:
            continue

        name = entry_map[eid]
        t    = record.getTimestamp() / 1e6
        typ  = type_map[eid]
        try:
            if typ in ("double", "float"):
                val = record.getDouble()
            elif typ == "boolean":
                val = 1.0 if record.getBoolean() else 0.0
            elif typ == "int64":
                val = float(record.getInteger())
            else:
                continue
        except Exception:
            continue

        raw[name].append(val)
        raw_t[name].append(t)
        record_count += 1

    # ── Build teleop windows ────────────────────────────────────────────────
    # Zip timestamps back onto values for the state signals
    enabled_pts = list(zip(raw_t[TELEOP_SIGNAL], raw[TELEOP_SIGNAL]))
    auton_pts   = list(zip(raw_t[AUTON_SIGNAL],  raw[AUTON_SIGNAL]))

    all_change_times = sorted(
        set(t for t, _ in enabled_pts) | set(t for t, _ in auton_pts)
    )

    en_sorted = sorted(enabled_pts)
    au_sorted = sorted(auton_pts)
    tele_times = []
    tele_vals  = []
    en = au = 0.0
    ei = ai = 0
    for ct in all_change_times:
        while ei < len(en_sorted) and en_sorted[ei][0] <= ct:
            en = en_sorted[ei][1]; ei += 1
        while ai < len(au_sorted) and au_sorted[ai][0] <= ct:
            au = au_sorted[ai][1]; ai += 1
        tele_times.append(ct)
        tele_vals.append(en == 1.0 and au == 0.0)

    def in_teleop(t):
        idx = bisect.bisect_right(tele_times, t) - 1
        return idx >= 0 and tele_vals[idx]

    # ── Filter each signal to teleop and (optionally) downsample ───────────
    teleop_data   = {}     # sig -> list[float]  (downsampled)
    teleop_raw_bv = []     # (t, v) for battery voltage (all teleop samples for min)
    pivot_raw     = []     # (t, v) for pivot (all teleop samples for peak)

    for sig in CURRENT_SIGNALS:
        if sig not in raw:
            continue
        ts = raw_t[sig]
        vs = raw[sig]

        # Filter to teleop
        tele_vs = []
        tele_tvs = []
        for t, v in zip(ts, vs):
            if in_teleop(t):
                tele_vs.append(v)
                tele_tvs.append(t)

        if not tele_vs:
            continue

        # Keep full (t, v) lists for signals we need precision on
        bv_sig = "/SystemStats/BatteryVoltage"
        pv_sig = "/Intake/Pivot/Current"
        if sig == bv_sig:
            teleop_raw_bv = list(zip(tele_tvs, tele_vs))
        if sig == pv_sig:
            pivot_raw = list(zip(tele_tvs, tele_vs))

        # Downsample for stats table (avg is stable with 6k samples even from 600k)
        n = len(tele_vs)
        if n > MAX_RAW_SAMPLES_PER_SIGNAL:
            step = n // MAX_RAW_SAMPLES_PER_SIGNAL
            tele_vs = tele_vs[::step]

        teleop_data[sig] = tele_vs

    return {
        "teleop_data":    teleop_data,
        "teleop_raw_bv":  teleop_raw_bv,
        "pivot_raw":      pivot_raw,
    }


def summarize(parsed):
    """Return per-subsystem stats using pre-parsed data."""
    teleop_data = parsed["teleop_data"]

    subsystem_samples = defaultdict(list)
    for sig, vals in teleop_data.items():
        sub = CURRENT_SIGNALS.get(sig)
        if sub and not sub.startswith("_"):
            subsystem_samples[sub].extend(vals)

    stats = {}
    for sub, vals in subsystem_samples.items():
        if not vals:
            stats[sub] = {"avg": 0, "max": 0, "n": 0}
        else:
            stats[sub] = {
                "avg": sum(vals) / len(vals),
                "max": max(vals),
                "n":   len(vals),
            }

    # Battery stats from full-resolution data
    bv_pairs = parsed["teleop_raw_bv"]
    bv = [v for _, v in bv_pairs]
    bc = teleop_data.get("/SystemStats/BatteryCurrent", [])
    stats["_Battery"] = {
        "avg_voltage": sum(bv) / len(bv) if bv else 0,
        "min_voltage": min(bv) if bv else 0,
        "avg_current": sum(bc) / len(bc) if bc else 0,
        "max_current": max(bc) if bc else 0,
        "bv_pairs":    bv_pairs,
    }
    return stats


def get_match_type(filename):
    m = re.search(r'_azfg_(q|e|p)(\d+)', filename, re.IGNORECASE)
    if not m:
        return ("?", 0)
    return (m.group(1).upper(), int(m.group(2)))


# ── Main ────────────────────────────────────────────────────────────────────
seen = set()
match_logs = []
for d in LOG_DIRS:
    for f in os.listdir(d):
        if "_azfg_" in f and f.endswith(".wpilog") and f not in seen:
            seen.add(f)
            match_type, match_num = get_match_type(f)
            fpath = os.path.join(d, f)
            size_mb = os.path.getsize(fpath) / 1e6
            match_logs.append((match_type, match_num, f, fpath, size_mb))

match_logs.sort(key=lambda x: (x[0], x[1]))

SUBSYSTEMS = ["Drive", "IntakePivot", "IntakeRollers", "Turret", "Flywheel", "Hood", "Kicker", "Spindexer"]

p("=" * 90)
p(f"{'Match':<12} {'MB':>5} {'Drive':>7} {'IPivot':>7} {'IRoll':>7} {'Turret':>7} {'Flywheel':>9} {'Hood':>7} {'Kicker':>7} {'Spindex':>8} {'AvgV':>6} {'MinV':>6}")
p("=" * 90)

all_stats   = defaultdict(list)
parsed_cache = {}  # label -> parsed dict (so we never re-read a file)

for match_type, match_num, fname, fpath, size_mb in match_logs:
    label = f"{match_type}{match_num}"
    print(f"Processing {label} ({size_mb:.0f}MB)...", flush=True)
    try:
        parsed = parse_log(fpath)
        parsed_cache[label] = parsed
        stats  = summarize(parsed)

        row = f"{label:<12} {size_mb:>5.0f}"
        for sub in SUBSYSTEMS:
            avg = stats.get(sub, {}).get("avg", 0)
            all_stats[sub].append(avg)
            row += f" {avg:>7.2f}"

        batt  = stats.get("_Battery", {})
        avg_v = batt.get("avg_voltage", 0)
        min_v = batt.get("min_voltage", 0)
        row  += f" {avg_v:>6.2f} {min_v:>6.2f}"
        p(row)

    except Exception as ex:
        import traceback
        p(f"{label:<12}  ERROR: {ex}")
        traceback.print_exc()

p("=" * 90)

# Grand averages
row = f"{'AVERAGE':<12} {'':>5}"
for sub in SUBSYSTEMS:
    vals = all_stats[sub]
    row += f" {(sum(vals)/len(vals) if vals else 0):>7.2f}"
p(row)

p()
p("Units: all current in Amps (avg during teleop, drive = sum of 8 motor signals). Voltage in Volts.")
p()

# ── Brownout section (uses cached parsed data — no re-read) ─────────────────
p("=" * 90)
p("BROWNOUT RISK (battery voltage < 8V during teleop):")
p("=" * 90)
for match_type, match_num, fname, fpath, size_mb in match_logs:
    label = f"{match_type}{match_num}"
    try:
        parsed = parsed_cache.get(label)
        if parsed is None:
            p(f"  {label:<8}  SKIPPED (parse failed)")
            continue
        bv_pairs = parsed["teleop_raw_bv"]
        low = [(t, v) for t, v in bv_pairs if v < 8.0]
        if low:
            worst = min(low, key=lambda x: x[1])
            p(f"  {label:<8}  {len(low):4d} samples <8V  |  worst: {worst[1]:.2f}V at t={worst[0]:.1f}s")
        else:
            p(f"  {label:<8}  OK")
    except Exception as ex:
        p(f"  {label:<8}  ERROR: {ex}")

p()

# ── Pivot stall section (uses cached parsed data — no re-read) ──────────────
p("=" * 90)
p("INTAKE PIVOT — peak current per match (stall indicator):")
p("=" * 90)
for match_type, match_num, fname, fpath, size_mb in match_logs:
    label = f"{match_type}{match_num}"
    try:
        parsed = parsed_cache.get(label)
        if parsed is None:
            p(f"  {label:<8}  SKIPPED")
            continue
        pv = parsed["pivot_raw"]
        if pv:
            vals = [v for _, v in pv]
            peak = max(vals)
            high = sum(1 for v in vals if v > 20)
            pct  = 100 * high // len(vals) if vals else 0
            p(f"  {label:<8}  peak={peak:6.1f}A   samples>20A: {high:5d} ({pct:2d}% of teleop)")
        else:
            p(f"  {label:<8}  NO DATA")
    except Exception as ex:
        p(f"  {label:<8}  ERROR: {ex}")

_out.close()
print(f"\nResults also saved to: {output_path}")
