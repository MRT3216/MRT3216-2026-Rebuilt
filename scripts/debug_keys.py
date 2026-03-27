#!/usr/bin/env python3
"""Quick debug: dump all key names from the Idaho match log."""
import struct, os

LOG = r"D:\logs\akit_26-03-27_01-50-13_idbo.wpilog"

with open(LOG, "rb") as f:
    header = f.read(12)
    ver = struct.unpack("<HI", header[6:12])
    extra_len = struct.unpack("<I", f.read(4))[0]
    f.read(extra_len)
    buf = f.read(1024 * 1024)  # first 1MB only

pos = 0
length = len(buf)
keys_found = []
while pos < length:
    if pos + 4 > length: break
    hdr1 = struct.unpack_from("<I", buf, pos)[0]
    entry_id = hdr1 & 0x00FFFFFF
    payload_size_len = (hdr1 >> 24) & 0x03
    type_indicator = (hdr1 >> 26) & 0x01
    ts_size = 8 if (hdr1 >> 27) & 1 == 0 else 4
    pos += 4
    if pos + ts_size > length: break
    timestamp = struct.unpack_from("<q" if ts_size == 8 else "<I", buf, pos)[0]
    pos += ts_size
    if payload_size_len == 0:
        if pos + 1 > length: break
        payload_size = buf[pos]; pos += 1
    elif payload_size_len == 1:
        if pos + 2 > length: break
        payload_size = struct.unpack_from("<H", buf, pos)[0]; pos += 2
    elif payload_size_len == 2:
        if pos + 4 > length: break
        payload_size = struct.unpack_from("<I", buf, pos)[0]; pos += 4
    else:
        break
    if pos + payload_size > length: break
    payload = buf[pos:pos + payload_size]
    pos += payload_size
    
    if type_indicator == 1:
        ctrl_type = payload[0] if payload else 255
        if ctrl_type == 0 and len(payload) >= 17:
            eid = struct.unpack_from("<I", payload, 1)[0]
            name_len = struct.unpack_from("<I", payload, 5)[0]
            name = payload[9:9 + name_len].decode("utf-8", errors="replace")
            type_len = struct.unpack_from("<I", payload, 9 + name_len)[0]
            type_str = payload[13 + name_len:13 + name_len + type_len].decode("utf-8", errors="replace")
            keys_found.append((eid, name, type_str))
            if "Cycle" in name or "MS" in name or "UserCode" in name or "GCTime" in name:
                print(f"  eid={eid:4d}  type={type_str:20s}  name={name}")

print(f"\nTotal keys in first 1MB: {len(keys_found)}")
# Print any key containing LoggedRobot or Logger
for eid, name, ts in keys_found:
    if "LoggedRobot" in name or "Logger" in name:
        print(f"  eid={eid:4d}  type={ts:20s}  name={name}")
