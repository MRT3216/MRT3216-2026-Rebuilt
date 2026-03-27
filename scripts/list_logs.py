"""List all .wpilog and .revlog files in ~/Desktop/Logs2 recursively, with sizes and dates."""
import os
import datetime

root = os.path.expanduser("~/Desktop/Logs2")
print(f"Scanning: {root}\n")

logs = []
for dirpath, dirnames, filenames in os.walk(root):
    for f in filenames:
        if f.endswith((".wpilog", ".revlog")):
            full = os.path.join(dirpath, f)
            size = os.path.getsize(full)
            mtime = os.path.getmtime(full)
            rel = os.path.relpath(full, root)
            logs.append((rel, size, mtime))

logs.sort(key=lambda x: x[2])

print(f"{'File':<75s}  {'Size':>10s}  {'Modified'}")
print("-" * 110)
for rel, size, mtime in logs:
    dt = datetime.datetime.fromtimestamp(mtime)
    size_str = f"{size/1024/1024:.1f} MB" if size > 1024*1024 else f"{size/1024:.1f} KB"
    print(f"{rel:<75s}  {size_str:>10s}  {dt.strftime('%Y-%m-%d %H:%M')}")

print(f"\nTotal: {len(logs)} log files")
