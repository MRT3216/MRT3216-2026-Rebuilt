"""Quick Q46 CANivore disconnect analysis — scan for Connected=false patterns."""
import struct, os

LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "q46.wpilog")

# ── Minimal WPILib DataLog reader ──
class WPILogReader:
    def __init__(self, path):
        self.f = open(path, "rb")
        magic = self.f.read(6)
        assert magic == b"WPILOG", f"Bad magic: {magic}"
        ver = struct.unpack("<H", self.f.read(2))[0]
        extra_len = struct.unpack("<I", self.f.read(4))[0]
        self.f.read(extra_len)  # skip extra header
        self.entries = {}  # id -> name

    def records(self):
        while True:
            hdr = self.f.read(1)
            if not hdr:
                return
            bit_len = hdr[0]
            if bit_len == 0:
                # Read entry length byte count
                len_bytes = self.f.read(1)
                if not len_bytes:
                    return
                entry_len = len_bytes[0]
            else:
                entry_len = bit_len

            # Actually the format is more nuanced. Let me use a proper approach.
            self.f.seek(0)
            break

    def close(self):
        self.f.close()


# Use the datalog reader from wpilib if available
try:
    from wpiutil.log import DataLogReader
    HAS_WPIUTIL = True
except ImportError:
    HAS_WPIUTIL = False

if not HAS_WPIUTIL:
    # Try robotpy
    try:
        from wpiutil.log import DataLogReader
        HAS_WPIUTIL = True
    except ImportError:
        pass

if not HAS_WPIUTIL:
    print("wpiutil not found, trying pip install...")
    import subprocess
    subprocess.check_call(["pip", "install", "robotpy-wpiutil", "-q"])
    try:
        from wpiutil.log import DataLogReader
        HAS_WPIUTIL = True
    except ImportError:
        # Last resort: try pyntcore
        subprocess.check_call(["pip", "install", "pyntcore", "-q"])
        from wpiutil.log import DataLogReader
        HAS_WPIUTIL = True

print(f"Reading {LOG} ({os.path.getsize(LOG)/1e6:.1f} MB)...")
reader = DataLogReader(LOG)

# Pass 1: Find all entry IDs and their names
entries = {}
for record in reader:
    if record.isStart():
        data = record.getStartData()
        entries[data.entry] = data.name

# Look for connected/disconnect signals
print(f"\nTotal entries: {len(entries)}")
print("\n--- Connection / Error related keys ---")
keywords = ["connect", "fault", "error", "bus", "overrun", "watchdog",
            "warning", "disconnect", "canivore", "can/"]
connected_keys = {}
for eid, name in sorted(entries.items(), key=lambda x: x[1]):
    name_lower = name.lower()
    if any(kw in name_lower for kw in keywords):
        print(f"  [{eid}] {name}")
        connected_keys[eid] = name

# Pass 2: Read connected signals for disconnect events
print("\n--- Scanning for disconnect events ---")
reader2 = DataLogReader(LOG)
disconnects = []
last_state = {}

for record in reader2:
    if record.isStart():
        continue
    if record.isFinish():
        continue
    if record.isSetMetadata():
        continue

    eid = record.getEntry()
    name = entries.get(eid, "")
    name_lower = name.lower()

    # Look for boolean Connected signals going false
    if "connected" in name_lower or "driveconnected" in name_lower or "turnconnected" in name_lower:
        try:
            val = record.getBoolean()
            ts = record.getTimestamp() / 1e6  # microseconds -> seconds
            prev = last_state.get(eid)
            if prev is not None and prev != val:
                event = "DISCONNECTED" if not val else "reconnected"
                disconnects.append((ts, name, event))
                print(f"  t={ts:8.2f}s  {name}: {event}")
            last_state[eid] = val
        except Exception:
            pass

    # Also look for loop overrun / watchdog
    if "loopoverrun" in name_lower or "watchdog" in name_lower:
        try:
            val = record.getBoolean()
            ts = record.getTimestamp() / 1e6
            if val:
                print(f"  t={ts:8.2f}s  {name}: OVERRUN")
        except Exception:
            pass

if not disconnects:
    print("  No disconnect events found in boolean signals.")
    print("\n  Searching for string/alert entries...")
    reader3 = DataLogReader(LOG)
    for record in reader3:
        if record.isStart() or record.isFinish() or record.isSetMetadata():
            continue
        eid = record.getEntry()
        name = entries.get(eid, "")
        if "alert" in name.lower() or "error" in name.lower():
            try:
                val = record.getString()
                if val and ("can" in val.lower() or "disconnect" in val.lower()):
                    ts = record.getTimestamp() / 1e6
                    print(f"  t={ts:8.2f}s  {name}: {val}")
            except Exception:
                pass

print(f"\nTotal disconnect/reconnect events: {len(disconnects)}")
if disconnects:
    print("\n--- Timeline ---")
    for ts, name, event in sorted(disconnects):
        marker = "🔴" if event == "DISCONNECTED" else "🟢"
        print(f"  {marker} {ts:8.2f}s  {name}: {event}")
