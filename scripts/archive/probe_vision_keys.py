"""Quick probe: list all vision-related log keys in the Idaho wpilog."""
import os, glob
from wpiutil.log import DataLogReader

log_dir = os.path.expanduser("~/Desktop/Logs2")
# Find the Idaho log folder (contains $)
for d in os.listdir(log_dir):
    if "Idaho" in d or "idaho" in d:
        log_dir = os.path.join(log_dir, d)
        break

# Find the wpilog
for f in os.listdir(log_dir):
    if f.endswith(".wpilog"):
        log_path = os.path.join(log_dir, f)
        break

print(f"Log: {log_path}")
keys = {}
for r in DataLogReader(log_path):
    if r.isStart():
        d = r.getStartData()
        keys[d.name] = d.type

vk = {k: v for k, v in sorted(keys.items()) if any(x in k.lower() for x in ["vision", "camera", "photon"])}
print(f"\n{len(vk)} vision-related keys:")
for k, t in sorted(vk.items()):
    print(f"  [{t:>12s}] {k}")

# Also check for any other keys with "Cam" or "cam"
ck = {k: v for k, v in sorted(keys.items()) if "cam" in k.lower() or "Cam" in k}
extra = {k: v for k, v in ck.items() if k not in vk}
if extra:
    print(f"\nAdditional 'cam' keys:")
    for k, t in sorted(extra.items()):
        print(f"  [{t:>12s}] {k}")
