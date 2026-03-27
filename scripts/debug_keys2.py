#!/usr/bin/env python3
"""Debug wpilog header and first few records to verify parsing."""
import struct

LOG = r"D:\logs\akit_26-03-27_01-50-13_idbo.wpilog"

with open(LOG, "rb") as f:
    magic = f.read(6)
    print(f"Magic: {magic}")
    version = struct.unpack("<H", f.read(2))[0]
    print(f"Version: {version}")
    
    if version >= 0x0100:
        # v1.0+ format
        extra_len = struct.unpack("<I", f.read(4))[0]
        extra = f.read(extra_len)
        print(f"Extra header length: {extra_len}")
        print(f"Extra header: {extra[:100]}...")
    
    # Read first few raw bytes to understand record format
    raw = f.read(256)
    print(f"\nFirst 256 bytes after header:")
    for i in range(0, min(256, len(raw)), 16):
        hex_part = " ".join(f"{b:02x}" for b in raw[i:i+16])
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in raw[i:i+16])
        print(f"  {i:4d}: {hex_part:<48s}  {ascii_part}")
    
    # Try the standard wpilog record format used by AdvantageKit
    # Records: 1-byte header bit, then entry ID (varint), timestamp (int64), payload
    # Actually let me use the wpiutil library which can parse wpilog natively
    
print("\n--- Trying wpiutil DataLogReader ---")
try:
    from wpiutil.log import DataLogReader
    reader = DataLogReader(LOG)
    count = 0
    timing_entries = {}
    for record in reader:
        if record.isStart():
            data = record.getStartData()
            name = data.name
            etype = data.type
            eid = data.entry
            if "Cycle" in name or "MS" in name or "UserCode" in name or "LoggedRobot" in name or "Logger/" in name:
                print(f"  eid={eid:4d}  type={etype:20s}  name={name}")
                timing_entries[eid] = name
        count += 1
        if count > 5000:
            break
    print(f"\nProcessed {count} records, found {len(timing_entries)} timing entries")
except Exception as e:
    print(f"wpiutil failed: {e}")
