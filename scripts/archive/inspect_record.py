#!/usr/bin/env python3
"""Inspect wpiutil.log DataLogRecord attributes."""
from wpiutil.log import DataLogReader
import struct

LOG = r"D:\logs\akit_26-03-27_01-50-13_idbo.wpilog"
reader = DataLogReader(LOG)
count = 0
for record in reader:
    if count == 0:
        print("Record type:", type(record))
        print("Dir:", [x for x in dir(record) if not x.startswith('_')])
    if record.isStart():
        sd = record.getStartData()
        print(f"Start: entry={sd.entry} name={sd.name} type={sd.type}")
        if count > 3:
            break
    else:
        if count < 5:
            print(f"Data: isControl={record.isControl()} timestamp={record.timestamp}")
            print(f"  hasData: entry attr? {'entry' in dir(record)}")
            try:
                print(f"  getEntry: {record.getEntry() if hasattr(record, 'getEntry') else 'N/A'}")
            except:
                pass
            # Check for getData methods
            data_methods = [x for x in dir(record) if 'data' in x.lower() or 'entry' in x.lower() or 'get' in x.lower()]
            print(f"  data methods: {data_methods}")
    count += 1
    if count > 10:
        break
