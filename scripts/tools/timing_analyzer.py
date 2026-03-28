#!/usr/bin/env python3
"""
timing_analyzer.py — Break down loop timing from AdvantageKit WPILogs.

Shows where CPU time is spent per phase (AUTO/TELEOP): UserCode, LogPeriodic,
GC, and sub-component breakdown. Works on single files or directories.

Usage:
    python -m tools.timing_analyzer match.wpilog
    python -m tools.timing_analyzer "C:/Logs/"
    python -m tools.timing_analyzer match.wpilog --budget 20
"""

import argparse
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tools.wpilog_utils import LogReader, extract_match_label


# AdvantageKit timing keys
TIMING_KEYS = {
    "FullCycle":    "/RealOutputs/LoggedRobot/FullCycleMS",
    "UserCode":     "/RealOutputs/LoggedRobot/UserCodeMS",
    "LogPeriodic":  "/RealOutputs/LoggedRobot/LogPeriodicMS",
    "GCTime":       "/RealOutputs/LoggedRobot/GCTimeMS",
    "GCCounts":     "/RealOutputs/LoggedRobot/GCCounts",
    "AutoLog":      "/RealOutputs/Logger/AutoLogMS",
    "CondCapture":  "/RealOutputs/Logger/ConduitCaptureMS",
    "CondSave":     "/RealOutputs/Logger/ConduitSaveMS",
    "Console":      "/RealOutputs/Logger/ConsoleMS",
    "DashInputs":   "/RealOutputs/Logger/DashboardInputsMS",
    "DS":           "/RealOutputs/Logger/DriverStationMS",
    "EntryUpdate":  "/RealOutputs/Logger/EntryUpdateMS",
    "AlertLog":     "/RealOutputs/Logger/AlertLogMS",
    "RadioLog":     "/RealOutputs/Logger/RadioLogMS",
}

LOGGER_SUB_KEYS = [
    "AutoLog", "CondCapture", "CondSave", "Console",
    "DashInputs", "DS", "EntryUpdate", "AlertLog", "RadioLog",
]


def get_timing_values(lr: LogReader, key_name: str, phase_ranges: list) -> list[float]:
    """Get timing values for a key filtered to phase ranges."""
    full_key = TIMING_KEYS.get(key_name)
    if not full_key:
        return []
    data = lr.get_timeseries(full_key)
    vals = []
    for ts, raw_val in data:
        if not any(s <= ts <= e for s, e in phase_ranges):
            continue
        # Handle AK int64-encoded doubles
        if isinstance(raw_val, int):
            import struct
            try:
                val = struct.unpack("d", struct.pack("q", raw_val))[0]
            except Exception:
                continue
        elif isinstance(raw_val, float):
            val = raw_val
        else:
            continue
        if 0 <= val < 50000:  # sanity
            vals.append(val)
    return vals


def analyze_timing(lr: LogReader, budget_ms: float = 20.0) -> dict:
    """Analyze loop timing for all active phases."""
    phases = lr.get_phases()
    results = {}

    for phase_name in ["AUTO", "TELEOP"]:
        ranges = [(s, e) for n, s, e in phases if n == phase_name]
        if not ranges:
            continue

        phase_data = {}
        for key_name in TIMING_KEYS:
            vals = get_timing_values(lr, key_name, ranges)
            if vals:
                n = len(vals)
                phase_data[key_name] = {
                    "mean": statistics.mean(vals),
                    "median": statistics.median(vals),
                    "p95": sorted(vals)[int(0.95 * n)] if n > 1 else vals[0],
                    "p99": sorted(vals)[int(0.99 * n)] if n > 1 else vals[0],
                    "max": max(vals),
                    "n": n,
                }

        if phase_data:
            fc = phase_data.get("FullCycle", {})
            overruns = sum(1 for v in get_timing_values(lr, "FullCycle", ranges) if v > budget_ms)
            total = phase_data.get("FullCycle", {}).get("n", 0)

            results[phase_name] = {
                "data": phase_data,
                "overruns": overruns,
                "total_cycles": total,
                "overrun_pct": 100 * overruns / total if total > 0 else 0,
                "duration": sum(e - s for s, e in ranges),
            }

    return results


def print_timing_report(label: str, results: dict, budget_ms: float = 20.0):
    """Print formatted timing report."""
    for phase_name, pr in results.items():
        pd = pr["data"]

        print(f"\n{'='*85}")
        print(f"  {label} — {phase_name} TIMING ({pr['duration']:.1f}s, "
              f"{pr['overruns']}/{pr['total_cycles']} overruns = {pr['overrun_pct']:.1f}%)")
        print(f"{'='*85}")

        print(f"  {'Component':>15s} | {'Mean':>7s} | {'Median':>7s} | "
              f"{'P95':>7s} | {'P99':>7s} | {'Max':>8s} | {'N':>6s}")
        print(f"  {'-'*15}-+-{'-'*7}-+-{'-'*7}-+-{'-'*7}-+-{'-'*7}-+-{'-'*8}-+-{'-'*6}")

        # Sort by mean descending, show only main keys
        main_keys = ["FullCycle", "UserCode", "LogPeriodic", "GCTime"]
        for key in main_keys:
            if key in pd:
                d = pd[key]
                print(f"  {key:>15s} | {d['mean']:6.2f}ms | {d['median']:6.2f}ms | "
                      f"{d['p95']:6.2f}ms | {d['p99']:6.2f}ms | {d['max']:7.2f}ms | {d['n']:6d}")

        # Budget analysis
        fc = pd.get("FullCycle", {})
        uc = pd.get("UserCode", {})
        lp = pd.get("LogPeriodic", {})
        gc = pd.get("GCTime", {})

        if fc and uc and lp:
            fc_m = fc["mean"]
            uc_m = uc["mean"]
            lp_m = lp["mean"]
            gc_m = gc["mean"] if gc else 0

            print(f"\n  WHERE IS THE TIME GOING:")
            print(f"    Full cycle:    {fc_m:>6.2f}ms")
            print(f"    = UserCode:    {uc_m:>6.2f}ms  ({100*uc_m/fc_m:.0f}%)")
            print(f"    + LogPeriodic: {lp_m:>6.2f}ms  ({100*lp_m/fc_m:.0f}%)")
            print(f"    + GC:          {gc_m:>6.3f}ms")
            print(f"    + Other:       {fc_m - uc_m - lp_m:>6.2f}ms")
            print(f"    Budget:        {budget_ms:>6.2f}ms")
            print(f"    Over budget:   {fc_m - budget_ms:>+6.2f}ms")

        # Logger sub-breakdown
        logger_total = 0
        logger_parts = []
        for key in LOGGER_SUB_KEYS:
            if key in pd:
                m = pd[key]["mean"]
                logger_total += m
                logger_parts.append((key, m, pd[key]["p95"], pd[key]["max"]))

        if logger_parts:
            print(f"\n  LOGGER SUB-BREAKDOWN:")
            for key, m, p95, mx in sorted(logger_parts, key=lambda x: -x[1]):
                print(f"    {key:>15s}: {m:6.3f}ms  (P95={p95:.3f}ms, max={mx:.2f}ms)")
            print(f"    {'TOTAL':>15s}: {logger_total:6.3f}ms")

        # GC analysis
        if gc and gc["n"] > 0:
            gc_vals = get_timing_values(LogReader.__new__(LogReader), "GCTime", []) if False else []
            # Just use the stats we already have
            print(f"\n  GC: mean={gc['mean']:.3f}ms, max={gc['max']:.2f}ms")


def print_batch_table(all_results: dict, budget_ms: float = 20.0):
    """Print comparison table across matches."""
    print(f"\n{'Match':>5s}  {'Phase':>7s}  {'Dur':>5s}  {'Mean':>7s}  {'P95':>7s}  "
          f"{'Max':>8s}  {'Overruns':>10s}  {'UserCode':>10s}  {'LogPer':>8s}")
    print(f"{'─'*5}  {'─'*7}  {'─'*5}  {'─'*7}  {'─'*7}  {'─'*8}  {'─'*10}  {'─'*10}  {'─'*8}")

    for label, results in all_results.items():
        for phase, pr in results.items():
            pd = pr["data"]
            fc = pd.get("FullCycle", {})
            uc = pd.get("UserCode", {})
            lp = pd.get("LogPeriodic", {})
            print(f"{label:>5s}  {phase:>7s}  {pr['duration']:>4.0f}s  "
                  f"{fc.get('mean',0):>6.2f}m  {fc.get('p95',0):>6.2f}m  "
                  f"{fc.get('max',0):>7.1f}ms  "
                  f"{pr['overrun_pct']:>5.1f}%  "
                  f"{uc.get('mean',0):>6.2f}ms  {lp.get('mean',0):>6.2f}ms")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze loop timing from AdvantageKit WPILogs.",
    )
    parser.add_argument("path", help="Path to a .wpilog file or directory")
    parser.add_argument("--budget", type=float, default=20.0,
                        help="Loop budget in ms (default: 20)")
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

    print(f"Analyzing {len(logs)} log(s), budget={args.budget}ms\n")

    all_results = {}
    for label, log_path in logs:
        print(f"  {label}: {log_path.name}...", end="", flush=True)
        try:
            lr = LogReader(str(log_path))
            r = analyze_timing(lr, budget_ms=args.budget)
            if r:
                all_results[label] = r
                phases = ", ".join(f"{p}: {d['overrun_pct']:.0f}% overruns" for p, d in r.items())
                print(f" {phases}")
            else:
                print(" no timing data")
        except Exception as e:
            print(f" ERROR: {e}")

    if not all_results:
        print("No logs analyzed.")
        sys.exit(1)

    if len(all_results) == 1:
        label, results = next(iter(all_results.items()))
        print_timing_report(label, results, budget_ms=args.budget)
    else:
        print_batch_table(all_results, budget_ms=args.budget)
        # Also print detailed for first match as example
        label, results = next(iter(all_results.items()))
        print(f"\n  (Detailed breakdown for {label}:)")
        print_timing_report(label, results, budget_ms=args.budget)

    print()


if __name__ == "__main__":
    main()
