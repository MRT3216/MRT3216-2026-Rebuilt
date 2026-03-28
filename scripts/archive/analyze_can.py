#!/usr/bin/env python3
"""
analyze_can.py — General-purpose WPILog CAN / disconnect analyzer for MRT3216.

Usage:
    python analyze_can.py <path_to.wpilog> [options]

Options:
    --no-timeline       Suppress the full event timeline
    --csv               Output disconnect counts as CSV
    --min-gap SECONDS   Ignore initial disconnects before this timestamp (default: 15s)

Examples:
    python analyze_can.py "C:/Logs/match_q46.wpilog"
    python analyze_can.py ./q55.wpilog --no-timeline
    python analyze_can.py ./q52.wpilog --min-gap 5 --csv
"""

import argparse
import os
import sys
from collections import Counter, defaultdict
from pathlib import Path

try:
    from wpiutil.log import DataLogReader
except ImportError:
    print("ERROR: robotpy-wpiutil is required.  Install with:")
    print("  pip install robotpy-wpiutil")
    sys.exit(1)

# ── Module / camera identity maps (MRT3216 2026 robot) ──────────────────────
MODULE_NAMES = {
    "Module0": "Front Left  (FL) — CAN 11/12/13",
    "Module1": "Front Right (FR) — CAN 21/22/23",
    "Module2": "Back Left   (BL) — CAN 41/42/43",
    "Module3": "Back Right  (BR) — CAN 31/32/33",
}

CAMERA_NAMES = {
    "Camera0": "Front_Cam",
    "Camera1": "Left_Cam",
    "Camera2": "Right_Cam",
    "Camera3": "Rear_Cam",
}

SIDE_MAP = {
    "Module0": "LEFT",
    "Module1": "RIGHT",
    "Module2": "LEFT",
    "Module3": "RIGHT",
    "Camera0": "CENTER",
    "Camera1": "LEFT",
    "Camera2": "RIGHT",
    "Camera3": "CENTER",
}


def classify_device(name: str) -> tuple[str, str, str]:
    """Return (short_key, friendly_name, side) for a /Connected entry name."""
    for key, friendly in MODULE_NAMES.items():
        if key in name:
            sub = name.split(key + "/")[-1].replace("Connected", "").rstrip("/") or "Module"
            return f"{key}/{sub}", f"{key} {sub} ({friendly})", SIDE_MAP[key]
    for key, friendly in CAMERA_NAMES.items():
        if key in name:
            return key, f"{key} ({friendly})", SIDE_MAP[key]
    if "OrangePI-Left" in name:
        return "OrangePI-Left", "OrangePI-Left (coprocessor)", "LEFT"
    if "OrangePI-Right" in name:
        return "OrangePI-Right", "OrangePI-Right (coprocessor)", "RIGHT"
    if "Pigeon" in name:
        return "Pigeon2", "Pigeon2 IMU (CAN 10)", "CENTER"
    # Generic fallback
    short = name.replace("/Connected", "").split("/")[-1]
    return short, name, "?"


def parse_log(path: str):
    """Parse a .wpilog file and return structured data."""
    reader = DataLogReader(path)

    # Pass 1: build entry map
    entries = {}
    for record in reader:
        if record.isStart():
            d = record.getStartData()
            entries[d.entry] = (d.name, d.type)

    # Identify CAN bus stat entry IDs
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

    # Pass 2: collect data
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

        # CAN bus stats
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

        # Connected booleans
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

    return entries, disconnects, dict(can_data)


def print_header(title: str, width: int = 72):
    print(f"\n{'=' * width}")
    print(f"  {title}")
    print(f"{'=' * width}")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze WPILog files for CAN bus disconnects and errors.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("logfile", help="Path to a .wpilog file")
    parser.add_argument("--no-timeline", action="store_true", help="Suppress full event timeline")
    parser.add_argument("--csv", action="store_true", help="Output disconnect counts as CSV")
    parser.add_argument(
        "--min-gap",
        type=float,
        default=15.0,
        help="Ignore disconnects before this time (seconds). Default: 15",
    )
    args = parser.parse_args()

    logfile = Path(args.logfile).resolve()
    if not logfile.exists():
        print(f"ERROR: File not found: {logfile}")
        sys.exit(1)

    size_mb = logfile.stat().st_size / 1e6
    print(f"Reading {logfile.name} ({size_mb:.1f} MB)...")

    entries, disconnects, can_data = parse_log(str(logfile))
    print(f"Log entries: {len(entries)}")

    # ── Filter early boot events ─────────────────────────────────────────
    boot_events = [d for d in disconnects if d[0] < args.min_gap]
    match_events = [d for d in disconnects if d[0] >= args.min_gap]
    disc_only = [d for d in match_events if d[2] == "DISCONNECTED"]

    # ── Timeline ─────────────────────────────────────────────────────────
    if not args.no_timeline:
        print_header(f"EVENT TIMELINE — {len(disconnects)} total ({len(boot_events)} during boot, {len(match_events)} in match)")
        for ts, name, event in sorted(disconnects):
            marker = "🔴" if event == "DISCONNECTED" else "🟢"
            boot_tag = " [boot]" if ts < args.min_gap else ""
            print(f"  {marker} {ts:8.2f}s  {name}: {event}{boot_tag}")

    # ── Disconnect counts ────────────────────────────────────────────────
    disc_counts = Counter()
    for _, name, _ in disc_only:
        disc_counts[name] += 1

    print_header(f"DISCONNECT COUNTS ({len(disc_only)} in-match disconnects)")

    if args.csv:
        print("  count,device,friendly_name,side")
        for name, count in disc_counts.most_common():
            _, friendly, side = classify_device(name)
            print(f"  {count},{name},{friendly},{side}")
    else:
        for name, count in disc_counts.most_common():
            _, friendly, side = classify_device(name)
            bar = "█" * count
            print(f"  {count:3d}x  {name}")
            print(f"       ↳ {friendly}  [{side}]  {bar}")

    # ── Side-of-robot summary ────────────────────────────────────────────
    side_counts = defaultdict(int)
    for _, name, _ in disc_only:
        _, _, side = classify_device(name)
        side_counts[side] += 1

    print_header("DISCONNECTS BY ROBOT SIDE")
    for side in ["RIGHT", "LEFT", "CENTER", "?"]:
        if side in side_counts:
            bar = "█" * side_counts[side]
            print(f"  {side:8s}: {side_counts[side]:3d}  {bar}")

    # ── Cascade detection ────────────────────────────────────────────────
    print_header("CASCADE CLUSTERS (disconnects within 3s of each other)")
    sorted_disc = sorted(disc_only, key=lambda d: d[0])
    clusters = []
    current_cluster = []
    for d in sorted_disc:
        if current_cluster and d[0] - current_cluster[-1][0] > 3.0:
            if len(current_cluster) >= 2:
                clusters.append(current_cluster)
            current_cluster = []
        current_cluster.append(d)
    if len(current_cluster) >= 2:
        clusters.append(current_cluster)

    if clusters:
        for i, cluster in enumerate(clusters, 1):
            t_start = cluster[0][0]
            t_end = cluster[-1][0]
            devices = set()
            for _, name, _ in cluster:
                short, _, _ = classify_device(name)
                devices.add(short)
            print(f"  Cluster {i}: t={t_start:.1f}–{t_end:.1f}s  ({len(cluster)} events, {len(devices)} devices)")
            for _, name, _ in cluster:
                _, friendly, side = classify_device(name)
                print(f"    • {friendly}  [{side}]")
    else:
        print("  No cascade clusters detected.")

    # ── Downtime per device ──────────────────────────────────────────────
    print_header("TOTAL DOWNTIME PER DEVICE (estimated)")
    # Track intervals where device is disconnected
    sorted_all = sorted(match_events, key=lambda d: d[0])
    device_disconnect_time = {}  # name -> timestamp of disconnect
    device_downtime = defaultdict(float)

    for ts, name, event in sorted_all:
        if event == "DISCONNECTED":
            device_disconnect_time[name] = ts
        elif event == "reconnected" and name in device_disconnect_time:
            dt = ts - device_disconnect_time.pop(name)
            device_downtime[name] += dt

    # Any still-disconnected at end of log
    for name, start_ts in device_disconnect_time.items():
        # Use last timestamp in log as end
        if sorted_all:
            device_downtime[name] += sorted_all[-1][0] - start_ts

    if device_downtime:
        for name, total_s in sorted(device_downtime.items(), key=lambda x: -x[1]):
            _, friendly, side = classify_device(name)
            print(f"  {total_s:6.2f}s  {name}")
            print(f"          ↳ {friendly}  [{side}]")
    else:
        print("  No measurable downtime.")

    # ── CAN Bus Stats ────────────────────────────────────────────────────
    print_header("CAN BUS STATS (roboRIO bus)")
    for key in ["off", "rx_err", "tx_err", "txfull"]:
        vals = can_data.get(key, [])
        if vals:
            max_val = max(v for _, v in vals)
            first_nz = next(((t, v) for t, v in vals if v > 0), None)
            line = f"  {key:10s}: max={max_val}, samples={len(vals)}"
            if first_nz:
                line += f", first non-zero at t={first_nz[0]:.2f}s"
            print(line)
        else:
            print(f"  {key:10s}: no data")

    util = can_data.get("util", [])
    if util:
        vals_only = [v for _, v in util]
        print(
            f"  {'util':10s}: avg={sum(vals_only)/len(vals_only):.1f}%"
            f", max={max(vals_only):.1f}%"
            f", min={min(vals_only):.1f}%"
        )

    # ── Verdict ──────────────────────────────────────────────────────────
    print_header("DIAGNOSIS")
    total_disc = len(disc_only)
    if total_disc == 0:
        print("  ✅ No in-match disconnects detected. CAN bus looks healthy.")
    else:
        print(f"  ⚠️  {total_disc} in-match disconnect events detected.\n")

        # Check if right-heavy
        r = side_counts.get("RIGHT", 0)
        l = side_counts.get("LEFT", 0)
        if r > l * 2 and r >= 5:
            print("  🔴 RIGHT-SIDE CONCENTRATION — Disconnects are heavily skewed to the right side.")
            print("     Check CAN wiring between FR (Module1) and BR (Module3) modules.")
            print("     Also check Right_Cam (Camera2) Ethernet cable.\n")
        elif l > r * 2 and l >= 5:
            print("  🔴 LEFT-SIDE CONCENTRATION — Disconnects are heavily skewed to the left side.")
            print("     Check CAN wiring between FL (Module0) and BL (Module2) modules.\n")

        # Check CAN errors
        max_rx = max((v for _, v in can_data.get("rx_err", [(0, 0)])), default=0)
        max_tx = max((v for _, v in can_data.get("tx_err", [(0, 0)])), default=0)
        max_off = max((v for _, v in can_data.get("off", [(0, 0)])), default=0)

        if max_rx > 0 or max_tx > 0:
            print(f"  🔴 CAN ERROR COUNTERS non-zero: rx_err={max_rx}, tx_err={max_tx}")
            print("     This indicates electrical noise or wiring issues on the roboRIO CAN bus.\n")
        elif max_off > 0:
            print(f"  🟡 CAN OffCount reached {max_off}. Some devices may have briefly gone bus-off.\n")
        else:
            print("  🟢 roboRIO CAN bus error counters are clean (0 errors).")
            print("     Disconnects are likely on the CANivore/CAN FD bus — check USB connection.\n")

        if clusters:
            biggest = max(clusters, key=len)
            print(f"  🟡 Largest cascade: {len(biggest)} events at t={biggest[0][0]:.1f}s.")
            print("     Cascading failures suggest a shared physical fault (connector/wire/USB).\n")

        # Compute total downtime
        total_down = sum(device_downtime.values())
        if total_down > 5.0:
            print(f"  🔴 Total device-seconds of downtime: {total_down:.1f}s")
        elif total_down > 1.0:
            print(f"  🟡 Total device-seconds of downtime: {total_down:.1f}s")

    print()


if __name__ == "__main__":
    main()
