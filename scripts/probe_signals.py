"""Quick signal name dump for one match log — used to confirm exact paths before building report2."""
from wpiutil.log import DataLogReader
import re

log_path = r"C:\Users\danla\Desktop\Logs2\akit_26-03-14_22-34-39_azfg_e9.wpilog"

names = {}
for record in DataLogReader(log_path):
    if record.isStart():
        d = record.getStartData()
        names[d.name] = d.type

# Print everything, grouped by prefix
groups = {}
for name, typ in sorted(names.items()):
    prefix = name.split("/")[1] if "/" in name else name
    groups.setdefault(prefix, []).append((name, typ))

KEYWORDS = [
    "command", "Command", "chassis", "Chassis", "speed", "Speed",
    "velocity", "Velocity", "flywheel", "Flywheel", "kicker", "Kicker",
    "spindex", "Spindex", "hood", "Hood", "loop", "Loop", "epoch", "Epoch",
    "vision", "Vision", "camera", "Camera", "battery", "Battery", "system", "System",
    "stats", "Stats", "Driver", "driver", "RealOutput", "realOutput"
]

print("=" * 70)
print(f"ALL SIGNALS matching keywords ({log_path.split(chr(92))[-1]}):")
print("=" * 70)
for name, typ in sorted(names.items()):
    if any(k.lower() in name.lower() for k in KEYWORDS):
        print(f"  {typ:<20} {name}")

print()
print("=" * 70)
print("ALL SIGNAL PREFIXES (top 2 levels):")
print("=" * 70)
prefixes = set()
for name in names:
    parts = name.strip("/").split("/")
    prefixes.add("/".join(parts[:2]))
for p in sorted(prefixes):
    print(f"  {p}")
