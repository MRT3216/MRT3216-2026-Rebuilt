"""Analyze a CANivore .hoot log for CAN bus errors and disconnects.

Uses phoenix6.HootReplay to iterate through the log and look for:
- Bus utilization spikes
- CAN error frames / transmit errors / receive errors
- Device disconnect patterns
- Timestamps of anomalies
"""

import os
import sys
from collections import defaultdict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CANIVORE_LOG = os.path.join(
    SCRIPT_DIR,
    "IDBO_Q46_DCA0D475463847532020204B182F0DFF_2026-03-28_01-01-33.hoot",
)
RIO_LOG = os.path.join(
    SCRIPT_DIR,
    "IDBO_Q46_rio_2026-03-28_01-01-33.hoot",
)

from phoenix6 import HootReplay

# ── Step 1: Inspect available signals in the CANivore hoot log ──
print("=" * 70)
print("CANIVORE HOOT LOG ANALYSIS")
print("=" * 70)

replay = HootReplay(CANIVORE_LOG)

# Discover what signals/entries are in the log
print("\n--- Available signals in CANivore log ---")
signals = replay.get_signals()
for i, sig in enumerate(signals):
    print(f"  [{i}] {sig}")
    if i > 100:
        print(f"  ... ({len(signals)} total signals, truncated)")
        break

print(f"\nTotal signals: {len(signals)}")

# Look specifically for bus-health-related signals
print("\n--- Bus health / error signals ---")
error_keywords = ["error", "fault", "bus", "utilization", "status", "disconnect",
                  "rx", "tx", "overrun", "overflow", "off", "warn"]
health_signals = []
for sig in signals:
    sig_lower = sig.lower()
    if any(kw in sig_lower for kw in error_keywords):
        health_signals.append(sig)
        print(f"  {sig}")

if not health_signals:
    print("  (none found matching error/bus keywords)")
    # Print all signals for manual inspection
    print("\n--- ALL signals (for manual inspection) ---")
    for sig in signals:
        print(f"  {sig}")

print("\n" + "=" * 70)
print("RIO HOOT LOG ANALYSIS")
print("=" * 70)

rio_replay = HootReplay(RIO_LOG)
rio_signals = rio_replay.get_signals()
print(f"\nTotal signals in RIO log: {len(rio_signals)}")
print("\n--- Bus health / error signals (RIO) ---")
for sig in rio_signals:
    sig_lower = sig.lower()
    if any(kw in sig_lower for kw in error_keywords):
        print(f"  {sig}")
