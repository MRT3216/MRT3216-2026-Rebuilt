#!/usr/bin/env python3
"""Check what commands are running during auto and look at drive output patterns."""
import struct
from wpiutil.log import DataLogReader

LOG = r"C:\Users\danla\Desktop\Logs Idaho\akit_26-03-28_16-55-43_idbo_q61.wpilog"

def rd(rec):
    try: return rec.getDouble()
    except:
        try: return struct.unpack('d', struct.pack('q', rec.getInteger()))[0]
        except: return None

print("Reading Q61...")
reader = DataLogReader(LOG)
entries = {}
for rec in reader:
    if rec.isStart():
        d = rec.getStartData()
        entries[d.entry] = (d.name, d.type)

# Find command-related and drive-related entries
print("\n--- Command entries ---")
cmd_entries = {}
for eid, (name, typ) in sorted(entries.items(), key=lambda x: x[1][0]):
    if "CommandsAll" in name or "CommandsUnique" in name:
        if "Intake" not in name and "Shooter" not in name and "Spindexer" not in name:
            print(f"  [{typ:10s}] {name}")
            cmd_entries[eid] = name

# Find drive outputs
print("\n--- Drive-related outputs ---")
drive_entries = {}
for eid, (name, typ) in sorted(entries.items(), key=lambda x: x[1][0]):
    nl = name.lower()
    if "/drive/" in nl and ("speed" in nl or "velocity" in nl or "setpoint" in nl or "output" in nl or "measured" in nl):
        if "odometry" not in nl and typ in ["double", "float", "int64"]:
            print(f"  [{typ:10s}] {name}")
            drive_entries[eid] = name

# Check for PathPlanner entries
print("\n--- PathPlanner related entries ---")
pp_entries = {}
for eid, (name, typ) in sorted(entries.items(), key=lambda x: x[1][0]):
    nl = name.lower()
    if "pathplanner" in nl or "pathfind" in nl or "followpath" in nl or "activeauto" in nl or "/auto/" in nl:
        print(f"  [{typ:10s}] {name}")
        pp_entries[eid] = name

# Also check for pose/odometry to see if the robot moved smoothly
print("\n--- Odometry/Pose entries ---")
pose_entries = {}
for eid, (name, typ) in sorted(entries.items(), key=lambda x: x[1][0]):
    if name in ["/RealOutputs/Odometry/Robot", "/RealOutputs/Drive/Pose"]:
        print(f"  [{typ:10s}] {name}")
        pose_entries[eid] = name

# Now collect command transitions during auto
AUTO_START = 238.80
AUTO_END = 246.75

reader2 = DataLogReader(LOG)
cmd_events = []
for rec in reader2:
    if rec.isStart() or rec.isFinish() or rec.isSetMetadata():
        continue
    eid = rec.getEntry()
    ts = rec.getTimestamp() / 1e6
    if 230 <= ts <= 250:
        if eid in cmd_entries:
            try:
                val = rec.getBoolean()
                cmd_events.append((ts, cmd_entries[eid], val))
            except:
                pass

print(f"\n{'='*90}")
print(f"COMMAND ACTIVITY AROUND AUTO (t=230-250s)")
print(f"{'='*90}")
for ts, name, val in sorted(cmd_events):
    short = name.split("/")[-1]
    state = "STARTED" if val else "ENDED"
    marker = " *** AUTO ***" if AUTO_START <= ts <= AUTO_END else ""
    print(f"  t={ts:8.2f}s  {state:8s}  {short}{marker}")
