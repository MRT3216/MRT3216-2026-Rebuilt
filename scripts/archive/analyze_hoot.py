"""Analyze a CANivore .hoot log for CAN bus errors, disconnects, and utilization."""

import sys
import os

# Add the scripts directory to path for the hoot files
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# The CANivore hoot log
CANIVORE_LOG = os.path.join(
    SCRIPT_DIR,
    "IDBO_Q46_DCA0D475463847532020204B182F0DFF_2026-03-28_01-01-33.hoot",
)
RIO_LOG = os.path.join(
    SCRIPT_DIR,
    "IDBO_Q46_rio_2026-03-28_01-01-33.hoot",
)

from phoenix6.unmanaged import feed_enable
from phoenix6 import SignalLogger

# Try to use the hoot log reader API
try:
    from phoenix6.sim_hoot_logger_manager import HootReplay
except ImportError:
    pass

# The phoenix6 Python library may not expose hoot reading directly.
# Let's try the lower-level approach using the native bindings.
try:
    from phoenix6._init_hoot import *
except ImportError:
    pass

# Let's inspect what's actually available in the phoenix6 package
import phoenix6
print("=== phoenix6 package contents ===")
for attr in sorted(dir(phoenix6)):
    if not attr.startswith("_"):
        print(f"  {attr}")

# Check for hoot-specific modules
print("\n=== Looking for hoot-related modules ===")
import pkgutil
for importer, modname, ispkg in pkgutil.walk_packages(
    phoenix6.__path__, prefix="phoenix6."
):
    if "hoot" in modname.lower() or "log" in modname.lower() or "replay" in modname.lower():
        print(f"  {modname} (package={ispkg})")

# Try direct import of hoot replay
print("\n=== Trying HootReplay ===")
try:
    from phoenix6.hoot_replay import HootReplay
    print("  Found phoenix6.hoot_replay.HootReplay")
except ImportError as e:
    print(f"  Not available: {e}")

try:
    from phoenix6 import HootReplay
    print("  Found phoenix6.HootReplay")
except ImportError as e:
    print(f"  Not available: {e}")

# Check if there's a SignalLogger that can read
print("\n=== SignalLogger attributes ===")
for attr in sorted(dir(SignalLogger)):
    if not attr.startswith("_"):
        print(f"  {attr}")

# Try to see if we can read the file header at least
print("\n=== Hoot file header inspection ===")
with open(CANIVORE_LOG, "rb") as f:
    header = f.read(256)
    print(f"  File size: {os.path.getsize(CANIVORE_LOG):,} bytes")
    print(f"  First 64 bytes (hex): {header[:64].hex()}")
    print(f"  First 64 bytes (ascii): {header[:64]}")
    # Look for magic bytes or version info
    print(f"  Looks like: ", end="")
    if header[:4] == b"HOOT":
        print("HOOT format confirmed")
    elif header[:4] == b"WLOG":
        print("WPILib log format")
    else:
        print(f"Unknown magic: {header[:4]}")

print(f"\n  RIO log size: {os.path.getsize(RIO_LOG):,} bytes")
with open(RIO_LOG, "rb") as f:
    header = f.read(64)
    print(f"  First 64 bytes (hex): {header[:64].hex()}")
