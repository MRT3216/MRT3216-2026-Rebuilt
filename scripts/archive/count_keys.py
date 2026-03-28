#!/usr/bin/env python3
"""
Count logged keys per wpilog to measure logging volume.
Compare BAD vs GOOD sessions to find if key count changed.
"""
from wpiutil.log import DataLogReader
import os, glob

LOG_DIRS = [os.path.expanduser("~/Desktop/Logs2"), r"D:\logs"]

LOGS_TO_CHECK = [
    ("akit_26-03-25_00-53-10.wpilog", "Early-3/25 (BAD)"),
    ("akit_26-03-26_02-51-20.wpilog", "3/26-morning (GOOD)"),
    ("akit_26-03-27_01-50-13_idbo.wpilog", "IDAHO MATCH"),
]

def find_log(basename):
    for d in LOG_DIRS:
        for f in glob.glob(os.path.join(d, "**", basename), recursive=True):
            return f
    return None

for basename, label in LOGS_TO_CHECK:
    path = find_log(basename)
    if not path:
        print(f"  SKIP: {basename}")
        continue
    
    reader = DataLogReader(path)
    keys = {}
    data_records = 0
    for record in reader:
        if record.isStart():
            sd = record.getStartData()
            keys[sd.entry] = sd.name
        elif not record.isControl():
            data_records += 1
    
    # Count keys by prefix
    prefixes = {}
    for name in keys.values():
        parts = name.split("/")
        if len(parts) >= 3:
            prefix = "/".join(parts[:3])
        else:
            prefix = name
        prefixes[prefix] = prefixes.get(prefix, 0) + 1
    
    print(f"\n{basename} -- {label}")
    print(f"  Total keys: {len(keys)}")
    print(f"  Total data records: {data_records:,}")
    print(f"  Data records per key: {data_records/len(keys):,.0f}")
    print(f"  Key count by prefix:")
    for prefix, count in sorted(prefixes.items(), key=lambda x: -x[1])[:20]:
        print(f"    {count:4d}  {prefix}")
