"""Q55 disconnect + CAN bus analysis — combined script for speed."""
import os
from wpiutil.log import DataLogReader

LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "q55.wpilog")
print(f"Reading {LOG} ({os.path.getsize(LOG)/1e6:.1f} MB)...")

# ── Pass 1: entry catalog ──
reader = DataLogReader(LOG)
entries = {}
for record in reader:
    if record.isStart():
        d = record.getStartData()
        entries[d.entry] = (d.name, d.type)

print(f"Total entries: {len(entries)}")

# ── Pass 2: disconnect events + CAN counters in one pass ──
reader2 = DataLogReader(LOG)

# CAN counter entry IDs
can_ids = {}
for eid, (name, _) in entries.items():
    if name == "/SystemStats/CANBus/OffCount": can_ids[eid] = "off"
    elif name == "/SystemStats/CANBus/ReceiveErrorCount": can_ids[eid] = "rx_err"
    elif name == "/SystemStats/CANBus/TransmitErrorCount": can_ids[eid] = "tx_err"
    elif name == "/SystemStats/CANBus/TxFullCount": can_ids[eid] = "txfull"
    elif name == "/SystemStats/CANBus/Utilization": can_ids[eid] = "util"

can_data = {k: [] for k in ["off", "rx_err", "tx_err", "txfull", "util"]}
disconnects = []
last_state = {}

for record in reader2:
    if record.isStart() or record.isFinish() or record.isSetMetadata():
        continue
    eid = record.getEntry()
    name = entries.get(eid, ("", ""))[0]
    name_lower = name.lower()

    # CAN counters
    if eid in can_ids:
        key = can_ids[eid]
        ts = record.getTimestamp() / 1e6
        try:
            val = record.getFloat() if key == "util" else record.getInteger()
            can_data[key].append((ts, val))
        except:
            try:
                can_data[key].append((ts, record.getDouble()))
            except:
                pass
        continue

    # Disconnect tracking
    if "connected" in name_lower:
        try:
            val = record.getBoolean()
            ts = record.getTimestamp() / 1e6
            prev = last_state.get(eid)
            if prev is not None and prev != val:
                event = "DISCONNECTED" if not val else "reconnected"
                disconnects.append((ts, name, event))
            last_state[eid] = val
        except:
            pass

# ── Print disconnect timeline ──
print(f"\n{'='*70}")
print(f"DISCONNECT EVENTS ({len(disconnects)} total)")
print(f"{'='*70}")
for ts, name, event in sorted(disconnects):
    marker = "🔴" if event == "DISCONNECTED" else "🟢"
    print(f"  {marker} {ts:8.2f}s  {name}: {event}")

# ── Summarize by device ──
from collections import Counter
disc_counts = Counter()
for _, name, event in disconnects:
    if event == "DISCONNECTED":
        disc_counts[name] += 1
print(f"\n{'='*70}")
print("DISCONNECT COUNTS BY DEVICE")
print(f"{'='*70}")
for name, count in disc_counts.most_common():
    print(f"  {count:3d}x  {name}")

# ── CAN bus stats ──
print(f"\n{'='*70}")
print("CAN BUS STATS (roboRIO bus)")
print(f"{'='*70}")
for key in ["off", "rx_err", "tx_err", "txfull"]:
    vals = can_data[key]
    if vals:
        max_val = max(v for _, v in vals)
        first_nz = next(((t, v) for t, v in vals if v > 0), None)
        print(f"  {key:10s}: max={max_val}, samples={len(vals)}" +
              (f", first non-zero at t={first_nz[0]:.2f}s" if first_nz else ""))
    else:
        print(f"  {key:10s}: no data")

util = can_data["util"]
if util:
    vals_only = [v for _, v in util]
    print(f"  {'util':10s}: avg={sum(vals_only)/len(vals_only):.1f}%, max={max(vals_only):.1f}%, min={min(vals_only):.1f}%")
