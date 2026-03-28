"""Quick scan of CAN bus error counters and utilization from Q46."""
import os
from wpiutil.log import DataLogReader

LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "q46.wpilog")
reader = DataLogReader(LOG)

entries = {}
for record in reader:
    if record.isStart():
        data = record.getStartData()
        entries[data.entry] = (data.name, data.type)

# Find CAN bus stats entries
target_keys = {
    "off": None, "rx_err": None, "tx_err": None,
    "txfull": None, "util": None
}
for eid, (name, typ) in entries.items():
    if name == "/SystemStats/CANBus/OffCount":
        target_keys["off"] = eid
    elif name == "/SystemStats/CANBus/ReceiveErrorCount":
        target_keys["rx_err"] = eid
    elif name == "/SystemStats/CANBus/TransmitErrorCount":
        target_keys["tx_err"] = eid
    elif name == "/SystemStats/CANBus/TxFullCount":
        target_keys["txfull"] = eid
    elif name == "/SystemStats/CANBus/Utilization":
        target_keys["util"] = eid

print("CAN Bus entry IDs:", target_keys)

# Read values
reader2 = DataLogReader(LOG)
can_data = {"off": [], "rx_err": [], "tx_err": [], "txfull": [], "util": []}
id_to_key = {v: k for k, v in target_keys.items() if v is not None}

for record in reader2:
    if record.isStart() or record.isFinish() or record.isSetMetadata():
        continue
    eid = record.getEntry()
    if eid in id_to_key:
        key = id_to_key[eid]
        ts = record.getTimestamp() / 1e6
        try:
            if key == "util":
                val = record.getFloat()
            else:
                val = record.getInteger()
            can_data[key].append((ts, val))
        except Exception:
            try:
                val = record.getDouble()
                can_data[key].append((ts, val))
            except:
                pass

# Print summary
for key in ["off", "rx_err", "tx_err", "txfull"]:
    vals = can_data[key]
    if vals:
        max_val = max(v for _, v in vals)
        last_val = vals[-1][1]
        # Find when it first went non-zero
        first_nonzero = next(((t, v) for t, v in vals if v > 0), None)
        print(f"\n{key}:")
        print(f"  Max: {max_val}, Final: {last_val}, Samples: {len(vals)}")
        if first_nonzero:
            print(f"  First non-zero at t={first_nonzero[0]:.2f}s (val={first_nonzero[1]})")
        # Show progression around the problem time (~274s)
        print(f"  Values around t=270-300s:")
        for t, v in vals:
            if 270 <= t <= 310:
                print(f"    t={t:.2f}s: {v}")
    else:
        print(f"\n{key}: No data")

# Utilization
util = can_data["util"]
if util:
    vals_only = [v for _, v in util]
    print(f"\nCAN Bus Utilization:")
    print(f"  Average: {sum(vals_only)/len(vals_only):.1f}%")
    print(f"  Max: {max(vals_only):.1f}%")
    print(f"  Min: {min(vals_only):.1f}%")
    # Show spikes above 50%
    spikes = [(t, v) for t, v in util if v > 50]
    if spikes:
        print(f"  Spikes above 50%: {len(spikes)}")
        for t, v in spikes[:20]:
            print(f"    t={t:.2f}s: {v:.1f}%")
    # Show values around problem time
    print(f"  Values around t=270-300s:")
    for t, v in util:
        if 270 <= t <= 310:
            print(f"    t={t:.2f}s: {v:.1f}%")
