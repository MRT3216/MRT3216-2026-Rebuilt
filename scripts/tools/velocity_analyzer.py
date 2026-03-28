#!/usr/bin/env python3
"""
velocity_analyzer.py — Analyze chassis velocity setpoints vs measured in WPILogs.

Detects PathPlanner velocity spirals by comparing commanded vs achieved speed
during autonomous. Works on a single log or a directory of logs.

Usage:
    python -m tools.velocity_analyzer <log_or_dir> [options]
    python -m tools.velocity_analyzer match.wpilog
    python -m tools.velocity_analyzer "C:/Logs/" --robot-max 6.02
    python -m tools.velocity_analyzer "C:/Logs/" --phase TELEOP

Options:
    --robot-max FLOAT   Max robot speed in m/s (default: 6.02 from TunerConstants)
    --phase NAME        Which phase to analyze: AUTO (default), TELEOP, or ALL
    --sample-rate FLOAT Sample interval in seconds (default: 0.25)
    --timeline          Print per-match velocity timeline
    --verbose           Show per-module breakdown
"""

import argparse
import math
import os
import sys
from pathlib import Path

# Allow running as: python -m tools.velocity_analyzer
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tools.wpilog_utils import LogReader, extract_match_label


def analyze_velocity(lr: LogReader, phase: str = "AUTO", robot_max: float = 6.02,
                     sample_rate: float = 0.25) -> dict | None:
    """Analyze velocity setpoints vs measured for a single log.

    Returns a dict with summary stats, or None if insufficient data.
    """
    if phase == "ALL":
        ranges = lr.get_phase_range("AUTO") + lr.get_phase_range("TELEOP")
    else:
        ranges = lr.get_phase_range(phase)

    if not ranges:
        return None

    overall_start = min(s for s, e in ranges)
    overall_end = max(e for s, e in ranges)

    cs_setp = [
        (ts, v) for ts, v in lr.get_timeseries("/RealOutputs/SwerveChassisSpeeds/Setpoints")
        if any(s <= ts <= e for s, e in ranges) and v is not None
    ]
    cs_meas = [
        (ts, v) for ts, v in lr.get_timeseries("/RealOutputs/SwerveChassisSpeeds/Measured")
        if any(s <= ts <= e for s, e in ranges) and v is not None
    ]

    if not cs_setp or not cs_meas:
        return None

    # Sample at regular intervals
    results = []
    t = overall_start
    while t <= overall_end:
        # Only sample if t is within a phase range
        in_range = any(s <= t <= e for s, e in ranges)
        if not in_range:
            t += sample_rate
            continue

        cs = min(cs_setp, key=lambda x: abs(x[0] - t))
        cm = min(cs_meas, key=lambda x: abs(x[0] - t))
        if abs(cs[0] - t) < sample_rate:
            sx, sy, _ = cs[1]
            mx, my, _ = cm[1]
            speed_s = math.hypot(sx, sy)
            speed_m = math.hypot(mx, my)
            err = abs(speed_s - speed_m)
            results.append((t - overall_start, speed_s, speed_m, err))
        t += sample_rate

    if not results:
        return None

    setpoints = [r[1] for r in results]
    errors = [r[3] for r in results]
    peak_setp = max(setpoints)
    avg_err = sum(errors) / len(errors)
    peak_err = max(errors)
    above_max = sum(1 for s in setpoints if s > robot_max)

    # Detect stutter periods (continuous >1.0 m/s error for >0.5s)
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

    # Module-level analysis
    mod_setp_raw = [
        (ts, v) for ts, v in lr.get_timeseries("/RealOutputs/SwerveStates/Setpoints")
        if any(s <= ts <= e for s, e in ranges) and v is not None
    ]
    mod_peaks = []
    mod_above = []
    if mod_setp_raw:
        for mod in range(min(4, len(mod_setp_raw[0][1]))):
            speeds = [abs(s[1][mod][0]) for s in mod_setp_raw if mod < len(s[1])]
            peak = max(speeds) if speeds else 0
            above = sum(1 for s in speeds if s > robot_max)
            mod_peaks.append(peak)
            mod_above.append(above)

    return {
        "duration": overall_end - overall_start,
        "peak_setp": peak_setp,
        "avg_setp": sum(setpoints) / len(setpoints),
        "avg_err": avg_err,
        "peak_err": peak_err,
        "above_max": above_max,
        "samples": len(results),
        "stutter_time": stutter_time,
        "stutters": stutters,
        "mod_peaks": mod_peaks,
        "mod_above": mod_above,
        "timeline": results,
    }


def print_summary_table(all_results: dict[str, dict], robot_max: float):
    """Print a comparison table across multiple matches."""
    print(f"\n{'Match':>5s}  {'Dur':>5s}  {'PeakSetp':>9s}  {'AvgErr':>7s}  {'PeakErr':>8s}  "
          f"{'>Max':>5s}  {'StutterT':>8s}  {'Verdict':>14s}")
    print(f"{'─'*5}  {'─'*5}  {'─'*9}  {'─'*7}  {'─'*8}  {'─'*5}  {'─'*8}  {'─'*14}")

    for label, r in all_results.items():
        verdict = ""
        if r["peak_setp"] > robot_max:
            verdict = "⚠️  OVER MAX"
        elif r["stutter_time"] > 1.0:
            verdict = "⚠️  STUTTER"
        elif r["avg_err"] > 1.0:
            verdict = "⚠️  HIGH ERR"
        elif r["avg_err"] > 0.5:
            verdict = "~  marginal"
        else:
            verdict = "✅ OK"

        print(f"{label:>5s}  {r['duration']:>4.0f}s  {r['peak_setp']:>8.2f}m  {r['avg_err']:>6.2f}m  "
              f"{r['peak_err']:>7.2f}m  {r['above_max']:>5d}  {r['stutter_time']:>7.1f}s  {verdict:>14s}")


def print_timeline(label: str, r: dict, robot_max: float, interval: float = 0.5):
    """Print velocity timeline for a single match."""
    print(f"\n  {label}:")
    print(f"    {'Time':>5s}  {'Setp':>6s}  {'Meas':>6s}  {'Err':>6s}")
    for t_rel, sp, ms, err in r["timeline"]:
        if t_rel % interval < 0.01 or abs(t_rel % interval - interval) < 0.01:
            flag = ""
            if sp > robot_max:
                flag = " ⚠️>MAX"
            elif err > 2.0:
                flag = " ❌"
            elif err > 1.0:
                flag = " ⚠️"
            print(f"    {t_rel:>5.1f}  {sp:>6.2f}  {ms:>6.2f}  {err:>6.2f}{flag}")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze chassis velocity setpoints vs measured in WPILog files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("path", help="Path to a .wpilog file or directory of .wpilog files")
    parser.add_argument("--robot-max", type=float, default=6.02,
                        help="Max robot speed in m/s (default: 6.02)")
    parser.add_argument("--phase", default="AUTO", choices=["AUTO", "TELEOP", "ALL"],
                        help="Phase to analyze (default: AUTO)")
    parser.add_argument("--sample-rate", type=float, default=0.25,
                        help="Sample interval in seconds (default: 0.25)")
    parser.add_argument("--timeline", action="store_true", help="Print per-match velocity timeline")
    parser.add_argument("--verbose", action="store_true", help="Show per-module breakdown")
    args = parser.parse_args()

    target = Path(args.path).resolve()

    # Collect log files
    if target.is_file():
        logs = [(extract_match_label(target.name), target)]
    elif target.is_dir():
        files = sorted(target.glob("*.wpilog"))
        if not files:
            print(f"No .wpilog files found in {target}")
            sys.exit(1)
        logs = [(extract_match_label(f.name), f) for f in files]
    else:
        print(f"Path not found: {target}")
        sys.exit(1)

    print(f"Analyzing {len(logs)} log(s), phase={args.phase}, robot_max={args.robot_max} m/s\n")

    all_results = {}
    for label, log_path in logs:
        print(f"  {label}: {log_path.name}...", end="", flush=True)
        try:
            lr = LogReader(str(log_path))
            r = analyze_velocity(lr, phase=args.phase, robot_max=args.robot_max,
                                 sample_rate=args.sample_rate)
            if r:
                all_results[label] = r
                print(f" peak={r['peak_setp']:.1f} m/s, err={r['avg_err']:.2f} m/s")
            else:
                print(" no data for this phase")
        except Exception as e:
            print(f" ERROR: {e}")

    if not all_results:
        print("\nNo matches with velocity data found.")
        sys.exit(1)

    # Summary table
    print_summary_table(all_results, args.robot_max)

    # Stutter periods
    any_stutters = any(r["stutters"] for r in all_results.values())
    if any_stutters:
        print(f"\nSTUTTER PERIODS (>1.0 m/s error for >0.5s):")
        for label, r in all_results.items():
            if r["stutters"]:
                print(f"  {label}: ", end="")
                for s, e, d in r["stutters"]:
                    print(f"[{s:.1f}–{e:.1f}s] ", end="")
                print()

    # Per-module breakdown
    if args.verbose:
        mod_names = ["FL", "FR", "BL", "BR"]
        print(f"\nPER-MODULE PEAK SETPOINTS:")
        for label, r in all_results.items():
            if r["mod_peaks"]:
                parts = [f"{mod_names[i]}={r['mod_peaks'][i]:.1f}" for i in range(len(r["mod_peaks"]))]
                print(f"  {label}: {', '.join(parts)}")

    # Timelines
    if args.timeline:
        print(f"\nVELOCITY TIMELINES:")
        for label, r in all_results.items():
            print_timeline(label, r, args.robot_max)

    print()


if __name__ == "__main__":
    main()
