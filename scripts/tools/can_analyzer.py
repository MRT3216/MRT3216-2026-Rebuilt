#!/usr/bin/env python3
"""
can_analyzer.py — Analyze CAN bus disconnects and errors in WPILog files.

Works on a single log or a directory for cross-match comparison.

Usage:
    python -m tools.can_analyzer match.wpilog
    python -m tools.can_analyzer "C:/Logs/"
    python -m tools.can_analyzer "C:/Logs/" --detail --min-gap 10

Options:
    --min-gap FLOAT   Ignore disconnects before this timestamp (default: 15s)
    --detail          Show per-match device breakdown (batch mode)
    --timeline        Show full event timeline (single file mode)
    --csv             Output as CSV
"""

import argparse
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tools.wpilog_utils import LogReader, classify_device, classify_side, extract_match_label


def analyze_can(lr: LogReader, boot_cutoff: float = 15.0) -> dict:
    """Analyze CAN disconnects for a single log. Returns summary dict."""

    # Find all /Connected keys
    connected_keys = [
        (name, typ) for name, typ, count in lr.list_keys(r"[Cc]onnected")
        if count > 0
    ]

    # CAN bus stat keys
    can_stat_map = {
        "/SystemStats/CANBus/OffCount": "off",
        "/SystemStats/CANBus/ReceiveErrorCount": "rx_err",
        "/SystemStats/CANBus/TransmitErrorCount": "tx_err",
        "/SystemStats/CANBus/TxFullCount": "txfull",
        "/SystemStats/CANBus/Utilization": "util",
    }

    # Collect disconnect events
    disconnects = []
    for name, typ, _ in lr.list_keys(r"[Cc]onnected"):
        ts_data = lr.get_timeseries(name)
        prev = None
        for ts, val in ts_data:
            if prev is not None and prev != val:
                event = "DISCONNECTED" if not val else "reconnected"
                disconnects.append((ts, name, event))
            prev = val

    # Split by boot window
    match_events = [d for d in disconnects if d[0] >= boot_cutoff]
    disc_only = [d for d in match_events if d[2] == "DISCONNECTED"]

    # Device counts
    device_counts = Counter()
    for _, name, _ in disc_only:
        device_counts[name] += 1

    # Side counts
    side_counts = defaultdict(int)
    for _, name, _ in disc_only:
        side_counts[classify_side(name)] += 1

    # Downtime
    sorted_events = sorted(match_events, key=lambda d: d[0])
    device_disconnect_time = {}
    device_downtime = defaultdict(float)
    for ts, name, event in sorted_events:
        if event == "DISCONNECTED":
            device_disconnect_time[name] = ts
        elif event == "reconnected" and name in device_disconnect_time:
            dt = ts - device_disconnect_time.pop(name)
            device_downtime[name] += dt
    if sorted_events:
        last_ts = sorted_events[-1][0]
        for name, start_ts in device_disconnect_time.items():
            device_downtime[name] += last_ts - start_ts

    # Cascade clusters (disconnects within 3s)
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

    # CAN stats
    can_stats = {}
    for full_key, short in can_stat_map.items():
        vals = lr.get_timeseries(full_key)
        if vals:
            numbers = [v for _, v in vals if isinstance(v, (int, float))]
            if numbers:
                can_stats[short] = {
                    "max": max(numbers),
                    "samples": len(numbers),
                }
                if short == "util":
                    can_stats[short]["avg"] = sum(numbers) / len(numbers)

    return {
        "total_disc": len(disc_only),
        "device_counts": device_counts,
        "side_counts": dict(side_counts),
        "device_downtime": dict(device_downtime),
        "total_downtime": sum(device_downtime.values()),
        "clusters": clusters,
        "can_stats": can_stats,
        "all_events": disconnects,
        "match_events": match_events,
    }


def print_single_report(label: str, r: dict, show_timeline: bool = False):
    """Print detailed report for a single match."""
    print(f"\n{'='*72}")
    print(f"  {label} — {r['total_disc']} in-match disconnects")
    print(f"{'='*72}")

    # Timeline
    if show_timeline and r["all_events"]:
        print(f"\n  EVENT TIMELINE ({len(r['all_events'])} total):")
        for ts, name, event in sorted(r["all_events"]):
            marker = "🔴" if event == "DISCONNECTED" else "🟢"
            short, friendly, side = classify_device(name)
            print(f"    {marker} {ts:8.2f}s  {short} [{side}]  {event}")

    # Device counts
    if r["device_counts"]:
        print(f"\n  DISCONNECT COUNTS:")
        for name, count in r["device_counts"].most_common():
            short, friendly, side = classify_device(name)
            bar = "█" * count
            print(f"    {count:3d}x  {short:<20s} [{side:>6s}]  {bar}")

    # Side summary
    if r["side_counts"]:
        print(f"\n  BY SIDE:")
        for side in ["RIGHT", "LEFT", "CENTER", "?"]:
            if side in r["side_counts"]:
                bar = "█" * r["side_counts"][side]
                print(f"    {side:>8s}: {r['side_counts'][side]:3d}  {bar}")

    # Cascade clusters
    if r["clusters"]:
        print(f"\n  CASCADE CLUSTERS ({len(r['clusters'])}):")
        for i, cluster in enumerate(r["clusters"], 1):
            t_start = cluster[0][0]
            t_end = cluster[-1][0]
            devices = set()
            for _, name, _ in cluster:
                devices.add(classify_device(name)[0])
            print(f"    Cluster {i}: t={t_start:.1f}–{t_end:.1f}s  "
                  f"({len(cluster)} events, {len(devices)} devices)")

    # CAN stats
    if r["can_stats"]:
        print(f"\n  CAN BUS STATS:")
        for key in ["off", "rx_err", "tx_err", "txfull", "util"]:
            if key in r["can_stats"]:
                s = r["can_stats"][key]
                line = f"    {key:>10s}: max={s['max']}"
                if "avg" in s:
                    line += f", avg={s['avg']:.1f}%"
                print(line)

    # Diagnosis
    print(f"\n  DIAGNOSIS:")
    if r["total_disc"] == 0:
        print("    ✅ No in-match disconnects. CAN bus looks healthy.")
    else:
        right = r["side_counts"].get("RIGHT", 0)
        left = r["side_counts"].get("LEFT", 0)
        if right > left * 2 and right >= 5:
            print("    🔴 RIGHT-SIDE CONCENTRATION — check CAN wiring FR↔BR + Right_Cam Ethernet.")
        elif left > right * 2 and left >= 5:
            print("    🔴 LEFT-SIDE CONCENTRATION — check CAN wiring FL↔BL.")
        if r["total_downtime"] > 5.0:
            print(f"    🔴 Total device-seconds of downtime: {r['total_downtime']:.1f}s")
        if r["clusters"]:
            biggest = max(r["clusters"], key=len)
            print(f"    🟡 Largest cascade: {len(biggest)} events — shared physical fault likely.")


def print_batch_table(all_results: dict[str, dict]):
    """Print comparison table across multiple matches."""
    print(f"\n{'Match':>5s}  {'Total':>5s}  {'RIGHT':>5s}  {'LEFT':>5s}  "
          f"{'Downtime':>8s}  {'Cascades':>8s}  {'Top Device':>25s}")
    print(f"{'─'*5}  {'─'*5}  {'─'*5}  {'─'*5}  {'─'*8}  {'─'*8}  {'─'*25}")

    for label, r in all_results.items():
        right = r["side_counts"].get("RIGHT", 0)
        left = r["side_counts"].get("LEFT", 0)
        top_dev = r["device_counts"].most_common(1)[0] if r["device_counts"] else ("—", 0)
        top_short = classify_device(top_dev[0])[0] if top_dev[0] != "—" else "—"
        print(f"{label:>5s}  {r['total_disc']:>5d}  {right:>5d}  {left:>5d}  "
              f"{r['total_downtime']:>7.1f}s  {len(r['clusters']):>8d}  "
              f"{top_short}×{top_dev[1]}")

    # Aggregate
    total_disc = sum(r["total_disc"] for r in all_results.values())
    total_right = sum(r["side_counts"].get("RIGHT", 0) for r in all_results.values())
    total_left = sum(r["side_counts"].get("LEFT", 0) for r in all_results.values())
    print(f"{'TOTAL':>5s}  {total_disc:>5d}  {total_right:>5d}  {total_left:>5d}")

    if total_disc > 0:
        pct_right = 100 * total_right / total_disc
        print(f"\n  RIGHT side: {pct_right:.0f}% of all disconnects")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze CAN bus disconnects in WPILog files.",
    )
    parser.add_argument("path", help="Path to a .wpilog file or directory")
    parser.add_argument("--min-gap", type=float, default=15.0,
                        help="Ignore disconnects before this time (default: 15s)")
    parser.add_argument("--detail", action="store_true",
                        help="Show per-match device breakdown (batch mode)")
    parser.add_argument("--timeline", action="store_true",
                        help="Show full event timeline")
    parser.add_argument("--csv", action="store_true", help="CSV output")
    args = parser.parse_args()

    target = Path(args.path).resolve()

    if target.is_file():
        logs = [(extract_match_label(target.name), target)]
    elif target.is_dir():
        files = sorted(target.glob("*.wpilog"))
        if not files:
            print(f"No .wpilog files in {target}")
            sys.exit(1)
        logs = [(extract_match_label(f.name), f) for f in files]
    else:
        print(f"Path not found: {target}")
        sys.exit(1)

    print(f"Analyzing {len(logs)} log(s) for CAN disconnects (boot_cutoff={args.min_gap}s)\n")

    all_results = {}
    for label, log_path in logs:
        print(f"  {label}: {log_path.name}...", end="", flush=True)
        try:
            lr = LogReader(str(log_path))
            r = analyze_can(lr, boot_cutoff=args.min_gap)
            all_results[label] = r
            print(f" {r['total_disc']} disconnects")
        except Exception as e:
            print(f" ERROR: {e}")

    if not all_results:
        print("No logs analyzed successfully.")
        sys.exit(1)

    if len(all_results) == 1:
        # Single file: detailed report
        label, r = next(iter(all_results.items()))
        print_single_report(label, r, show_timeline=args.timeline)
    else:
        # Batch: comparison table
        print_batch_table(all_results)
        if args.detail:
            for label, r in all_results.items():
                print_single_report(label, r, show_timeline=args.timeline)

    print()


if __name__ == "__main__":
    main()
