"""
Probe the REV log (.revlog) binary format to understand its structure.
"""
import struct
import os
import sys

LOG_PATH = r"C:\Users\danla\Desktop\Logs2\$Idaho\REV_20241218_140212.revlog"

with open(LOG_PATH, "rb") as f:
    data = f.read()

file_size = len(data)
print(f"File size: {file_size:,} bytes ({file_size/1024/1024:.2f} MB)")
print()

# ── Hex dump of first 256 bytes ──
print("=== First 256 bytes (hex) ===")
for i in range(0, 256, 16):
    hex_str = " ".join(f"{b:02x}" for b in data[i:i+16])
    ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in data[i:i+16])
    print(f"  {i:04x}: {hex_str}  {ascii_str}")
print()

# ── Look for ASCII strings anywhere in first 4KB ──
print("=== Scanning first 4KB for ASCII strings (len >= 6) ===")
i = 0
while i < min(4096, len(data)):
    if 32 <= data[i] < 127:
        j = i
        while j < min(4096, len(data)) and 32 <= data[j] < 127:
            j += 1
        if j - i >= 6:
            s = data[i:j].decode("ascii")
            print(f"  offset {i} (0x{i:04x}), len {j-i}: {repr(s)}")
        i = j
    else:
        i += 1
print()

# ── Try common header patterns ──
print("=== Header value guesses ===")
print(f"  Byte 0: 0x{data[0]:02x} = {data[0]}")
print(f"  Byte 1: 0x{data[1]:02x} = {data[1]}")
print(f"  Bytes 0-1 LE u16: {struct.unpack_from('<H', data, 0)[0]}")
print(f"  Bytes 0-3 LE u32: {struct.unpack_from('<I', data, 0)[0]}")
print(f"  Bytes 2-3 LE u16: {struct.unpack_from('<H', data, 2)[0]}")
print(f"  Bytes 4-7 LE u32: {struct.unpack_from('<I', data, 4)[0]}")
print()

# ── Check if the file is packed records with fixed-size frames ──
# Try to figure out record size by looking for repeating patterns
print("=== Looking for repeating record patterns ===")

# Check bytes 0-1 as potential record type/version marker
# Then try to find the same pattern later
marker = data[0:2]
print(f"  First 2 bytes (potential marker): {marker.hex()}")
for offset in range(2, min(100, len(data))):
    if data[offset:offset+2] == marker:
        print(f"  Marker found again at offset {offset} (record size = {offset}?)")
        break

# Try common record sizes (10, 12, 14, 16, 18, 20, 22, 24)
for rec_size in [8, 10, 12, 14, 16, 18, 20, 22, 24, 28, 32]:
    matches = 0
    for k in range(1, min(20, file_size // rec_size)):
        if data[k*rec_size] == data[0] and data[k*rec_size+1] == data[1]:
            matches += 1
    if matches >= 5:
        print(f"  Record size {rec_size}: {matches}/19 matches on first 2 bytes")
print()

# ── Byte frequency analysis of first 10KB ──
print("=== Byte value frequency (first 10KB, top 20) ===")
from collections import Counter
freq = Counter(data[:10240])
for val, count in freq.most_common(20):
    print(f"  0x{val:02x} ({val:3d}, {repr(chr(val)) if 32<=val<127 else '.':<5}): {count:5d} ({100*count/10240:.1f}%)")
print()

# ── Try REV's known telemetry frame format ──
# REV Hardware Client logs are known to use a format like:
#   [timestamp_ms: u32] [device_id: u8] [signal_count: u8] [signal_id: u8, value: varies]...
# or possibly MessagePack / protobuf

# Let's see if the first few bytes make sense as a version header
print("=== Attempting known REV log header ===")
# Byte 0 might be version
# Byte 1 might be a count or type
ver = data[0]
print(f"  Possible version: {ver}")

# After version, maybe device count or signal descriptor table
# Let's look at offset 2 as a potential u16 descriptor length
desc_len = struct.unpack_from('<H', data, 2)[0]
print(f"  Possible descriptor length at offset 2: {desc_len}")

# Try reading at offset after version byte(s) as a signal descriptor
# Maybe: [signal_id: u16] [name_len: u16] [name: bytes] ...
offset = 2
for attempt in range(5):
    if offset + 4 > len(data):
        break
    sig_id = struct.unpack_from('<H', data, offset)[0]
    name_len = struct.unpack_from('<H', data, offset+2)[0]
    if 0 < name_len < 200 and offset + 4 + name_len <= len(data):
        name_bytes = data[offset+4:offset+4+name_len]
        if all(32 <= b < 127 for b in name_bytes):
            print(f"  Signal descriptor at {offset}: id={sig_id}, name={name_bytes.decode('ascii')}")
            offset += 4 + name_len
            continue
    print(f"  No string at offset {offset}: sig_id={sig_id}, name_len={name_len}")
    break
print()

# ── Dump first 512 bytes in groups of various sizes ──
print("=== First 100 bytes as various int formats ===")
print("  As u8:  ", [data[i] for i in range(32)])
print("  As u16 LE:", [struct.unpack_from('<H', data, i*2)[0] for i in range(16)])
print("  As u32 LE:", [struct.unpack_from('<I', data, i*4)[0] for i in range(8)])
print("  As i16 LE:", [struct.unpack_from('<h', data, i*2)[0] for i in range(16)])
