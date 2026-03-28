"""Quick diagnostic: dump DriverStation state changes and signal counts."""
import sys
from collections import defaultdict
from wpiutil.log import DataLogReader

LOG = sys.argv[1]
entry_map = {}
type_map = {}

# Signals to track state
WATCH = {
    "/DriverStation/Enabled", "/DriverStation/Autonomous",
    "/DriverStation/MatchTime", "/DriverStation/MatchNumber",
    "/DriverStation/MatchType", "/DriverStation/FMSAttached",
    "/DriverStation/DSAttached", "/DriverStation/Alliance",
    "/DriverStation/AllianceStation",
}

data = defaultdict(list)
sig_counts = defaultdict(int)

for record in DataLogReader(LOG):
    if record.isStart():
        d = record.getStartData()
        entry_map[d.entry] = d.name
        type_map[d.entry] = d.type
        continue
    eid = record.getEntry()
    if eid not in entry_map:
        continue
    name = entry_map[eid]
    sig_counts[name] += 1
    if name not in WATCH:
        continue
    t = record.getTimestamp() / 1e6
    typ = type_map[eid]
    try:
        if typ == "boolean":
            val = record.getBoolean()
        elif typ in ("double", "float"):
            val = record.getDouble()
        elif typ == "int64":
            val = record.getInteger()
        elif typ == "string":
            val = record.getString()
        else:
            val = f"<{typ}>"
        data[name].append((t, val))
    except:
        pass

print("=== DriverStation Signal Transitions ===")
for name in sorted(data.keys()):
    pts = data[name]
    print(f"\n--- {name} ({len(pts)} samples) ---")
    if len(pts) <= 30:
        for t, v in pts:
            print(f"  t={t:.3f}s  val={v}")
    else:
        prev = None
        for t, v in pts:
            if v != prev:
                print(f"  t={t:.3f}s  val={v}")
                prev = v

print("\n=== Top 20 Signals by Sample Count ===")
for name, count in sorted(sig_counts.items(), key=lambda x: -x[1])[:20]:
    print(f"  {count:>8}  {name}")

print(f"\n=== Total unique signals: {len(sig_counts)} ===")
