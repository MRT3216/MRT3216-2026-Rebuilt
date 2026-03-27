"""Probe actual vision camera data in the Idaho wpilog — check Connected booleans,
pose observation counts, and alert text."""
import os, struct as st
from collections import defaultdict
from wpiutil.log import DataLogReader

log_dir = os.path.expanduser("~/Desktop/Logs2")
for d in os.listdir(log_dir):
    if "Idaho" in d or "idaho" in d:
        log_dir = os.path.join(log_dir, d)
        break
for f in os.listdir(log_dir):
    if f.endswith(".wpilog"):
        log_path = os.path.join(log_dir, f)
        break

print(f"Log: {log_path}\n")

entry_map = {}
type_map = {}
booleans = defaultdict(list)
string_arrays = defaultdict(list)
raw_entries = defaultdict(list)  # for struct types

for r in DataLogReader(log_path):
    if r.isStart():
        d = r.getStartData()
        entry_map[d.entry] = d.name
        type_map[d.entry] = d.type
        continue
    eid = r.getEntry()
    if eid not in entry_map:
        continue
    name = entry_map[eid]
    t = r.getTimestamp() / 1e6
    typ = type_map[eid]
    try:
        if typ == "boolean" and "Vision" in name:
            booleans[name].append((t, r.getBoolean()))
        elif typ == "string[]" and "Alert" in name:
            string_arrays[name].append((t, list(r.getStringArray())))
        elif "PoseObservation" in name:
            # Raw bytes — just track count and size
            raw = r.getRaw()
            raw_entries[name].append((t, len(raw)))
        elif name == "/RealOutputs/Vision/Summary/HasTarget":
            booleans[name].append((t, r.getBoolean()))
        elif name == "/RealOutputs/Vision/Summary/TagCount":
            booleans[name].append((t, r.getInteger()))
    except:
        pass

# ── Connected booleans ──
print("═" * 60)
print("  CAMERA CONNECTED STATUS")
print("═" * 60)
for cam in range(4):
    key = f"/Vision/Camera{cam}/Connected"
    pts = booleans.get(key, [])
    if not pts:
        print(f"  Camera {cam}: no Connected data logged")
        continue
    # Show transitions
    true_count = sum(1 for _, v in pts if v)
    false_count = sum(1 for _, v in pts if not v)
    print(f"  Camera {cam}: {len(pts)} samples — {true_count} true, {false_count} false")
    # Show first few transitions
    prev = None
    transitions = []
    for t, v in pts:
        if v != prev:
            transitions.append((t, v))
            prev = v
    for t, v in transitions[:10]:
        print(f"    t={t:.1f}s: Connected={'TRUE' if v else 'FALSE'}")
    if len(transitions) > 10:
        print(f"    ... ({len(transitions)} total transitions)")

# ── HasTarget summary ──
print(f"\n{'═' * 60}")
print("  HAS TARGET / TAG COUNT")
print("═" * 60)
ht = booleans.get("/RealOutputs/Vision/Summary/HasTarget", [])
if ht:
    true_count = sum(1 for _, v in ht if v)
    print(f"  HasTarget: {len(ht)} samples, {true_count} true ({100*true_count/len(ht):.1f}%)")
tc = booleans.get("/RealOutputs/Vision/Summary/TagCount", [])
if tc:
    vals = [v for _, v in tc]
    nonzero = sum(1 for v in vals if v > 0)
    print(f"  TagCount: {len(tc)} samples, {nonzero} nonzero ({100*nonzero/len(tc):.1f}%), max={max(vals)}")

# ── PoseObservation struct sizes (proxy for data presence) ──
print(f"\n{'═' * 60}")
print("  POSE OBSERVATIONS (raw struct data)")
print("═" * 60)
for cam in range(4):
    key = f"/Vision/Camera{cam}/PoseObservations"
    pts = raw_entries.get(key, [])
    if not pts:
        print(f"  Camera {cam}: no PoseObservation data")
        continue
    sizes = [s for _, s in pts]
    nonempty = sum(1 for s in sizes if s > 8)  # empty struct array still has length prefix
    print(f"  Camera {cam}: {len(pts)} records, {nonempty} with pose data ({100*nonempty/len(pts):.1f}%)")
    if nonempty > 0:
        nonempty_sizes = [s for s in sizes if s > 8]
        print(f"    Non-empty payload sizes: min={min(nonempty_sizes)}, max={max(nonempty_sizes)}, "
              f"mean={sum(nonempty_sizes)/len(nonempty_sizes):.0f}")

# ── Alerts about cameras ──
print(f"\n{'═' * 60}")
print("  VISION-RELATED ALERTS")
print("═" * 60)
cam_alerts = set()
photon_warnings = set()
for key in ["/RealOutputs/Alerts/warnings", "/RealOutputs/Alerts/errors",
            "/RealOutputs/PhotonAlerts/warnings", "/RealOutputs/PhotonAlerts/errors",
            "/RealOutputs/PhotonAlerts/infos"]:
    for t, arr in string_arrays.get(key, []):
        for w in arr:
            if any(x in w.lower() for x in ["camera", "vision", "photon", "disconnect"]):
                cam_alerts.add((key.split("/")[-1], w))

if cam_alerts:
    for level, msg in sorted(cam_alerts):
        print(f"  [{level}] {msg}")
else:
    print("  No vision/camera alerts found")

# ── RobotPoses struct sizes (proxy for pose data in output) ──
print(f"\n{'═' * 60}")
print("  ROBOT POSES OUTPUT (struct:Pose3d[])")
print("═" * 60)
for cam in range(4):
    for suffix in ["RobotPoses", "RobotPosesAccepted", "RobotPosesRejected"]:
        key = f"/RealOutputs/Vision/Camera{cam}/{suffix}"
        pts = raw_entries.get(key, [])
        if not pts:
            # These are struct types, check if we recorded them
            continue
        nonempty = sum(1 for _, s in pts if s > 0)
        print(f"  Camera{cam}/{suffix}: {len(pts)} records, {nonempty} non-empty")

# Re-read the struct pose arrays since we didn't capture them above
print("\n  (Re-scanning for struct:Pose3d[] in RealOutputs...)")
pose_counts = defaultdict(lambda: {"total": 0, "nonempty": 0})
for r in DataLogReader(log_path):
    if r.isStart():
        d = r.getStartData()
        entry_map[d.entry] = d.name
        type_map[d.entry] = d.type
        continue
    eid = r.getEntry()
    if eid not in entry_map:
        continue
    name = entry_map[eid]
    if "RealOutputs/Vision/Camera" in name and "Pose" in name:
        try:
            raw = r.getRaw()
            pose_counts[name]["total"] += 1
            if len(raw) > 0:
                pose_counts[name]["nonempty"] += 1
        except:
            pass

for key in sorted(pose_counts.keys()):
    c = pose_counts[key]
    print(f"  {key}: {c['total']} records, {c['nonempty']} non-empty ({100*c['nonempty']/max(c['total'],1):.0f}%)")
