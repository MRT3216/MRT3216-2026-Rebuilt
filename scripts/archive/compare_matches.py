#!/usr/bin/env python3
"""
compare_matches.py -- Batch-analyze all Idaho/Boise .wpilog files and produce
a cross-match comparison table.

Usage:
    python compare_matches.py <log_directory>
    python compare_matches.py <log_directory> --detail
"""

import argparse
import os
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

try:
    from wpiutil.log import DataLogReader
except ImportError:
    print("ERROR: robotpy-wpiutil is required.  pip install robotpy-wpiutil")
    sys.exit(1)

# ── Robot identity maps ──────────────────────────────────────────────────────
MODULE_NAMES = {
    "Module0": "FL",
    "Module1": "FR",
    "Module2": "BL",
    "Module3": "BR",
}

CAMERA_NAMES = {
    "Camera0": "Front",
    "Camera1": "Left",
    "Camera2": "Right",
    "Camera3": "Rear",
}

SIDE_MAP = {
    "Module0": "LEFT",  "Module1": "RIGHT",
    "Module2": "LEFT",  "Module3": "RIGHT",
    "Camera0": "CENTER","Camera1": "LEFT",
    "Camera2": "RIGHT", "Camera3": "CENTER",
}


def classify_side(name: str) -> str:
    for key, side in SIDE_MAP.items():
        if key in name:
            return side
    if "OrangePI-Left" in name:
        return "LEFT"
    if "OrangePI-Right" in name:
        return "RIGHT"
    return "?"


def short_device(name: str) -> str:
    """Return a compact device label like 'BR/TurnEnc' or 'Cam2/Right'."""
    for key, label in MODULE_NAMES.items():
        if key in name:
            sub = name.split(key + "/")[-1].replace("Connected", "").rstrip("/")
            sub = sub.replace("TurnEncoder", "TurnEnc")
            return f"{label}/{sub}" if sub else label
    for key, label in CAMERA_NAMES.items():
        if key in name:
            return f"Cam/{label}"
    if "OrangePI-Left" in name:
        return "OPi/Left"
    if "OrangePI-Right" in name:
        return "OPi/Right"
    if "Elastic" in name:
        return "Elastic"
    if "Pigeon" in name:
        return "Pigeon"
    return name.split("/")[-1].replace("Connected", "")[:15]


def extract_match_label(filename: str) -> str:
    """Extract 'Q6', 'Q55' etc from filename."""
    m = re.search(r'_q(\d+)', filename, re.IGNORECASE)
    return f"Q{m.group(1)}" if m else Path(filename).stem[:20]


def analyze_log(path: str, boot_cutoff: float = 15.0):
    """Analyze a single .wpilog and return a results dict."""
    reader = DataLogReader(path)

    entries = {}
    for record in reader:
        if record.isStart():
            d = record.getStartData()
            entries[d.entry] = (d.name, d.type)

    # CAN stat entry IDs
    can_stat_keys = {
        "/SystemStats/CANBus/OffCount": "off",
        "/SystemStats/CANBus/ReceiveErrorCount": "rx_err",
        "/SystemStats/CANBus/TransmitErrorCount": "tx_err",
        "/SystemStats/CANBus/TxFullCount": "txfull",
        "/SystemStats/CANBus/Utilization": "util",
    }
    can_ids = {}
    for eid, (name, _) in entries.items():
        if name in can_stat_keys:
            can_ids[eid] = can_stat_keys[name]

    can_data = defaultdict(list)
    disconnects = []
    last_state = {}

    reader2 = DataLogReader(path)
    for record in reader2:
        if record.isStart() or record.isFinish() or record.isSetMetadata():
            continue
        eid = record.getEntry()
        if eid not in entries:
            continue
        name = entries[eid][0]
        ts = record.getTimestamp() / 1e6

        if eid in can_ids:
            key = can_ids[eid]
            try:
                val = record.getFloat() if key == "util" else record.getInteger()
            except Exception:
                try:
                    val = record.getDouble()
                except Exception:
                    continue
            can_data[key].append((ts, val))
            continue

        if "connected" in name.lower():
            try:
                val = record.getBoolean()
            except Exception:
                continue
            prev = last_state.get(eid)
            if prev is not None and prev != val:
                event = "DISCONNECTED" if not val else "reconnected"
                disconnects.append((ts, name, event))
            last_state[eid] = val

    # Filter to in-match only
    match_events = [d for d in disconnects if d[0] >= boot_cutoff]
    disc_only = [d for d in match_events if d[2] == "DISCONNECTED"]

    # Per-device counts
    device_counts = Counter()
    for _, name, _ in disc_only:
        device_counts[name] += 1

    # Side counts
    side_counts = defaultdict(int)
    for _, name, _ in disc_only:
        side_counts[classify_side(name)] += 1

    # Downtime calculation
    sorted_events = sorted(match_events, key=lambda d: d[0])
    device_disconnect_time = {}
    device_downtime = defaultdict(float)
    for ts, name, event in sorted_events:
        if event == "DISCONNECTED":
            device_disconnect_time[name] = ts
        elif event == "reconnected" and name in device_disconnect_time:
            dt = ts - device_disconnect_time.pop(name)
            device_downtime[name] += dt
    # Still-disconnected at end
    if sorted_events:
        last_ts = sorted_events[-1][0]
        for name, start_ts in device_disconnect_time.items():
            device_downtime[name] += last_ts - start_ts

    # CAN error summary
    max_rx = max((v for _, v in can_data.get("rx_err", [(0, 0)])), default=0)
    max_tx = max((v for _, v in can_data.get("tx_err", [(0, 0)])), default=0)
    util_vals = [v for _, v in can_data.get("util", [])]
    avg_util = sum(util_vals) / len(util_vals) if util_vals else 0

    # Cascade clusters
    sorted_disc = sorted(disc_only, key=lambda d: d[0])
    clusters = []
    current = []
    for d in sorted_disc:
        if current and d[0] - current[-1][0] > 3.0:
            if len(current) >= 2:
                clusters.append(current)
            current = []
        current.append(d)
    if len(current) >= 2:
        clusters.append(current)

    return {
        "total_disc": len(disc_only),
        "device_counts": device_counts,
        "side_counts": dict(side_counts),
        "device_downtime": dict(device_downtime),
        "total_downtime": sum(device_downtime.values()),
        "max_rx_err": max_rx,
        "max_tx_err": max_tx,
        "avg_util": avg_util,
        "clusters": len(clusters),
        "biggest_cluster": max((len(c) for c in clusters), default=0),
    }


def print_header(title: str, width: int = 100):
    print(f"\n{'=' * width}")
    print(f"  {title}")
    print(f"{'=' * width}")


def main():
    parser = argparse.ArgumentParser(description="Batch compare Idaho match logs.")
    parser.add_argument("logdir", help="Directory containing .wpilog files")
    parser.add_argument("--detail", action="store_true", help="Show per-match device breakdown")
    parser.add_argument("--min-gap", type=float, default=15.0, help="Boot ignore window (default: 15s)")
    args = parser.parse_args()

    logdir = Path(args.logdir).resolve()
    if not logdir.is_dir():
        print(f"ERROR: Not a directory: {logdir}")
        sys.exit(1)

    wpilog_files = sorted(logdir.glob("*.wpilog"))
    if not wpilog_files:
        print(f"ERROR: No .wpilog files found in {logdir}")
        sys.exit(1)

    print(f"Found {len(wpilog_files)} match logs in {logdir.name}/\n")

    # ── Analyze all matches ──────────────────────────────────────────────
    results = {}
    all_devices = set()
    for f in wpilog_files:
        label = extract_match_label(f.name)
        size_mb = f.stat().st_size / 1e6
        print(f"  Analyzing {label} ({f.name}, {size_mb:.1f} MB)...", end="", flush=True)
        try:
            r = analyze_log(str(f), boot_cutoff=args.min_gap)
            results[label] = r
            all_devices.update(r["device_counts"].keys())
            print(f" {r['total_disc']} disconnects")
        except Exception as e:
            print(f" ERROR: {e}")

    if not results:
        print("No logs were successfully analyzed.")
        sys.exit(1)

    match_labels = list(results.keys())

    # ── Summary Table ────────────────────────────────────────────────────
    print_header("MATCH SUMMARY")
    col_w = 8
    header = f"  {'':20s}" + "".join(f"{m:>{col_w}s}" for m in match_labels)
    print(header)
    print(f"  {'-'*20}" + ("-" * col_w) * len(match_labels))

    # Total disconnects
    row = f"  {'Disconnects':20s}"
    for m in match_labels:
        v = results[m]["total_disc"]
        row += f"{v:>{col_w}d}"
    print(row)

    # Right side
    row = f"  {'Right side':20s}"
    for m in match_labels:
        v = results[m]["side_counts"].get("RIGHT", 0)
        row += f"{v:>{col_w}d}"
    print(row)

    # Left side
    row = f"  {'Left side':20s}"
    for m in match_labels:
        v = results[m]["side_counts"].get("LEFT", 0)
        row += f"{v:>{col_w}d}"
    print(row)

    # Center
    row = f"  {'Center':20s}"
    for m in match_labels:
        v = results[m]["side_counts"].get("CENTER", 0)
        row += f"{v:>{col_w}d}"
    print(row)

    # Cascade clusters
    row = f"  {'Cascade clusters':20s}"
    for m in match_labels:
        v = results[m]["clusters"]
        row += f"{v:>{col_w}d}"
    print(row)

    # Biggest cascade
    row = f"  {'Biggest cascade':20s}"
    for m in match_labels:
        v = results[m]["biggest_cluster"]
        row += f"{v:>{col_w}d}"
    print(row)

    # Total downtime
    row = f"  {'Downtime (dev·s)':20s}"
    for m in match_labels:
        v = results[m]["total_downtime"]
        row += f"{v:>{col_w}.0f}"
    print(row)

    # CAN errors
    row = f"  {'CAN rx_err max':20s}"
    for m in match_labels:
        v = results[m]["max_rx_err"]
        row += f"{v:>{col_w}}"
    print(row)

    row = f"  {'CAN tx_err max':20s}"
    for m in match_labels:
        v = results[m]["max_tx_err"]
        row += f"{v:>{col_w}}"
    print(row)

    row = f"  {'CAN util avg %':20s}"
    for m in match_labels:
        v = results[m]["avg_util"]
        row += f"{v:>{col_w}.1f}"
    print(row)

    # ── Per-Device Heatmap ───────────────────────────────────────────────
    print_header("DEVICE DISCONNECT HEATMAP (count per match)")

    # Build compact device list sorted by total
    device_totals = Counter()
    for m in match_labels:
        for dev, cnt in results[m]["device_counts"].items():
            device_totals[dev] += cnt

    # Filter to devices with at least 1 disconnect
    active_devices = [dev for dev, _ in device_totals.most_common() if device_totals[dev] > 0]

    col_w2 = 8
    header = f"  {'Device':25s} {'Side':6s}" + "".join(f"{m:>{col_w2}s}" for m in match_labels) + f"{'TOTAL':>{col_w2}s}"
    print(header)
    print(f"  {'-'*25} {'-'*6}" + ("-" * col_w2) * (len(match_labels) + 1))

    for dev in active_devices:
        short = short_device(dev)
        side = classify_side(dev)
        row = f"  {short:25s} {side:6s}"
        total = 0
        for m in match_labels:
            v = results[m]["device_counts"].get(dev, 0)
            total += v
            if v == 0:
                cell = "·"
            elif v <= 2:
                cell = str(v)
            elif v <= 5:
                cell = f"▪{v}"
            elif v <= 15:
                cell = f"▮{v}"
            else:
                cell = f"█{v}"
            row += f"{cell:>{col_w2}s}"
        row += f"{total:>{col_w2}d}"
        print(row)

    # ── Per-Device Downtime Heatmap ──────────────────────────────────────
    print_header("DEVICE DOWNTIME HEATMAP (seconds per match)")

    # Sort by total downtime
    downtime_totals = defaultdict(float)
    for m in match_labels:
        for dev, dt in results[m]["device_downtime"].items():
            downtime_totals[dev] += dt
    active_dt_devices = sorted(downtime_totals.keys(), key=lambda d: -downtime_totals[d])

    header = f"  {'Device':25s} {'Side':6s}" + "".join(f"{m:>{col_w2}s}" for m in match_labels) + f"{'TOTAL':>{col_w2}s}"
    print(header)
    print(f"  {'-'*25} {'-'*6}" + ("-" * col_w2) * (len(match_labels) + 1))

    for dev in active_dt_devices:
        short = short_device(dev)
        side = classify_side(dev)
        row = f"  {short:25s} {side:6s}"
        total = 0.0
        for m in match_labels:
            v = results[m]["device_downtime"].get(dev, 0.0)
            total += v
            if v == 0:
                cell = "·"
            elif v < 1:
                cell = f"{v:.1f}"
            else:
                cell = f"{v:.0f}"
            row += f"{cell:>{col_w2}s}"
        row += f"{total:>{col_w2}.0f}"
        print(row)

    # ── Per-Match Detail ─────────────────────────────────────────────────
    if args.detail:
        for m in match_labels:
            r = results[m]
            print_header(f"{m} — {r['total_disc']} disconnects, {r['total_downtime']:.0f}s downtime")
            for dev, cnt in r["device_counts"].most_common():
                short = short_device(dev)
                side = classify_side(dev)
                dt = r["device_downtime"].get(dev, 0)
                print(f"    {cnt:3d}x  {short:25s} [{side:6s}]  downtime={dt:.1f}s")

    # ── Cross-Match Diagnosis ────────────────────────────────────────────
    print_header("CROSS-MATCH DIAGNOSIS")

    total_all = sum(r["total_disc"] for r in results.values())
    total_right = sum(r["side_counts"].get("RIGHT", 0) for r in results.values())
    total_left = sum(r["side_counts"].get("LEFT", 0) for r in results.values())
    total_dt = sum(r["total_downtime"] for r in results.values())
    any_can_err = any(r["max_rx_err"] > 0 or r["max_tx_err"] > 0 for r in results.values())

    print(f"  Matches analyzed: {len(results)}")
    print(f"  Total disconnect events: {total_all}")
    print(f"  Total device-seconds downtime: {total_dt:.0f}")
    print()

    if total_all == 0:
        print("  ✅ No in-match disconnects across any log. CAN bus looks healthy!")
        print()
        return

    # Side analysis
    pct_right = total_right / total_all * 100 if total_all > 0 else 0
    pct_left = total_left / total_all * 100 if total_all > 0 else 0
    print(f"  Side distribution:  RIGHT={total_right} ({pct_right:.0f}%)  LEFT={total_left} ({pct_left:.0f}%)")

    if total_right > total_left * 2 and total_right >= 10:
        print()
        print("  🔴 PERSISTENT RIGHT-SIDE PROBLEM across all matches.")
        print("     This is a PHYSICAL wiring issue, not software.")
        print("     Priority actions:")
        print("       1. Check CANivore USB connection (hot glue / strain relief)")
        print("       2. Inspect CAN FD daisy chain on right side: FR→BR modules")
        print("       3. Check Right_Cam (Camera2) Ethernet cable")
    elif total_left > total_right * 2 and total_left >= 10:
        print()
        print("  🔴 PERSISTENT LEFT-SIDE PROBLEM across all matches.")
        print("     Check CAN wiring between FL (Module0) and BL (Module2).")

    # Trend analysis
    print()
    match_disc_list = [(m, results[m]["total_disc"]) for m in match_labels]
    first_half = match_disc_list[:len(match_disc_list)//2]
    second_half = match_disc_list[len(match_disc_list)//2:]
    avg_first = sum(v for _, v in first_half) / len(first_half) if first_half else 0
    avg_second = sum(v for _, v in second_half) / len(second_half) if second_half else 0

    if avg_second > avg_first * 1.5 and avg_second > 5:
        print(f"  📈 WORSENING TREND: Early matches avg {avg_first:.0f} disc → Later matches avg {avg_second:.0f} disc")
        print("     The problem is getting worse over time (connector degradation / loosening).")
    elif avg_first > avg_second * 1.5 and avg_first > 5:
        print(f"  📉 IMPROVING TREND: Early matches avg {avg_first:.0f} disc → Later matches avg {avg_second:.0f} disc")
        print("     Something was fixed or tightened between matches.")
    else:
        print(f"  ↔️  CONSISTENT: Early avg {avg_first:.0f} disc, Later avg {avg_second:.0f} disc")

    # Worst match
    worst_match = max(match_labels, key=lambda m: results[m]["total_disc"])
    worst_count = results[worst_match]["total_disc"]
    best_match = min(match_labels, key=lambda m: results[m]["total_disc"])
    best_count = results[best_match]["total_disc"]
    print(f"\n  Worst match: {worst_match} ({worst_count} disconnects, {results[worst_match]['total_downtime']:.0f}s downtime)")
    print(f"  Best match:  {best_match} ({best_count} disconnects, {results[best_match]['total_downtime']:.0f}s downtime)")

    # Top offending devices across all matches
    print()
    print("  Top offending devices (all matches combined):")
    for dev, total in device_totals.most_common(5):
        short = short_device(dev)
        side = classify_side(dev)
        dt = downtime_totals.get(dev, 0)
        print(f"    {total:4d}x  {short:25s} [{side:6s}]  total downtime={dt:.0f}s")

    if any_can_err:
        print("\n  🔴 CAN error counters were non-zero in at least one match — electrical noise issue.")
    else:
        print("\n  🟢 roboRIO CAN bus error counters clean in ALL matches.")
        print("     Problem is isolated to CANivore/CAN FD bus (USB + right-side wiring).")

    print()


if __name__ == "__main__":
    main()
