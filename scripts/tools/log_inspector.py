#!/usr/bin/env python3
"""
log_inspector.py — Inspect the contents of a WPILog file.

Lists all keys, their types, sample counts, and match phases.
Useful for discovering what's in a log before running other analyzers.

Usage:
    python -m tools.log_inspector match.wpilog
    python -m tools.log_inspector match.wpilog --filter "Drive"
    python -m tools.log_inspector match.wpilog --filter "Vision|Camera"
    python -m tools.log_inspector match.wpilog --types-only
    python -m tools.log_inspector match.wpilog --phases
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tools.wpilog_utils import LogReader


def main():
    parser = argparse.ArgumentParser(
        description="Inspect contents of a WPILog file.",
    )
    parser.add_argument("logfile", help="Path to a .wpilog file")
    parser.add_argument("--filter", default=None,
                        help="Regex filter for key names (e.g. 'Drive|Vision')")
    parser.add_argument("--types-only", action="store_true",
                        help="Group keys by type instead of listing individually")
    parser.add_argument("--phases", action="store_true",
                        help="Show match phase timeline")
    parser.add_argument("--sample", default=None,
                        help="Print first N samples for keys matching this regex")
    parser.add_argument("--sample-count", type=int, default=5,
                        help="Number of samples to print (default: 5)")
    args = parser.parse_args()

    path = Path(args.logfile).resolve()
    if not path.exists():
        print(f"File not found: {path}")
        sys.exit(1)

    size_mb = path.stat().st_size / 1e6
    print(f"Reading {path.name} ({size_mb:.1f} MB)...")

    lr = LogReader(str(path))

    keys = lr.list_keys(pattern=args.filter)
    print(f"Found {len(keys)} keys" + (f" matching '{args.filter}'" if args.filter else ""))

    # Phases
    if args.phases or True:  # always show phases
        phases = lr.get_phases()
        if phases:
            print(f"\nMatch phases:")
            for name, start, end in phases:
                if name != "DISABLED" or end - start > 1.0:
                    print(f"  {name:>10s}: {start:>8.2f}s – {end:>8.2f}s  ({end-start:.1f}s)")
        else:
            print("\nNo match phases detected.")

    # Type summary
    if args.types_only:
        type_counts = {}
        for name, typ, count in keys:
            type_counts.setdefault(typ, []).append((name, count))
        print(f"\nKeys by type:")
        for typ, entries in sorted(type_counts.items()):
            total_samples = sum(c for _, c in entries)
            print(f"\n  {typ} ({len(entries)} keys, {total_samples} total samples):")
            for name, count in sorted(entries):
                print(f"    {count:>6d}  {name}")
    else:
        # Full listing
        print(f"\n{'Type':<25s}  {'Samples':>8s}  {'Key'}")
        print(f"{'─'*25}  {'─'*8}  {'─'*50}")
        for name, typ, count in keys:
            print(f"{typ:<25s}  {count:>8d}  {name}")

    # Sample values
    if args.sample:
        import re
        sample_keys = [(n, t, c) for n, t, c in keys if re.search(args.sample, n, re.IGNORECASE)]
        if sample_keys:
            print(f"\nSample values for keys matching '{args.sample}':")
            for name, typ, count in sample_keys[:10]:
                data = lr.get_timeseries(name)[:args.sample_count]
                print(f"\n  {name} ({typ}):")
                for ts, val in data:
                    if isinstance(val, bytes):
                        print(f"    {ts:>10.3f}s  <{len(val)} bytes>")
                    elif isinstance(val, list):
                        print(f"    {ts:>10.3f}s  [{len(val)} items]")
                    else:
                        print(f"    {ts:>10.3f}s  {val}")

    # Summary stats
    total_samples = sum(c for _, _, c in keys)
    print(f"\nSummary: {len(keys)} keys, {total_samples:,} total samples")
    print()


if __name__ == "__main__":
    main()
