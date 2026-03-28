"""List all .wpilog files from both log directories, filtered to last 24 hours."""
import os
import datetime
import time

now = time.time()
cutoff = now - 24 * 3600  # last 24 hours

roots = [
    os.path.expanduser("~/Desktop/Logs2"),
    "D:\\logs",
]

logs = []
for root in roots:
    if not os.path.exists(root):
        print(f"⚠ Path not found: {root}")
        continue
    for dirpath, dirnames, filenames in os.walk(root):
        for f in filenames:
            if f.endswith(".wpilog"):
                full = os.path.join(dirpath, f)
                size = os.path.getsize(full)
                mtime = os.path.getmtime(full)
                if mtime >= cutoff:
                    logs.append((full, size, mtime))

logs.sort(key=lambda x: x[2])

print(f"Log files modified in last 24 hours (since {datetime.datetime.fromtimestamp(cutoff).strftime('%Y-%m-%d %H:%M')})")
print(f"{'#':>3s}  {'File':<90s}  {'Size':>10s}  {'Modified'}")
print("-" * 130)
for i, (full, size, mtime) in enumerate(logs):
    dt = datetime.datetime.fromtimestamp(mtime)
    size_str = f"{size/1024/1024:.1f} MB" if size > 1024*1024 else f"{size/1024:.1f} KB"
    print(f"{i+1:>3d}  {full:<90s}  {size_str:>10s}  {dt.strftime('%Y-%m-%d %H:%M')}")

print(f"\nTotal: {len(logs)} log files in last 24h")
