"""
Decode REV log records. Records are 16 bytes, starting at offset 15.
Header is bytes 0-14.

Record layout hypothesis (from byte frequency analysis):
  Positions 6,7=always near (b8,05) and 8,9=(02,00) - these look like a CAN ID
  
Let's figure out what each field means.
"""
import struct

LOG_PATH = r"C:\Users\danla\Desktop\Logs2\$Idaho\REV_20241218_140212.revlog"

with open(LOG_PATH, "rb") as f:
    data = f.read()

# Header: bytes 0-14
header = data[:15]
print("=== HEADER (15 bytes) ===")
print(" ".join(f"{b:02x}" for b in header))
print(f"  Bytes 0-1: {header[0]:02x} {header[1]:02x}  (version = {struct.unpack_from('<H', header, 0)[0]})")
print(f"  Byte 2: {header[2]:02x} = {header[2]}  (record type or something)")
print(f"  Bytes 3-6: {' '.join(f'{b:02x}' for b in header[3:7])}  (u32 LE = {struct.unpack_from('<I', header, 3)[0]})")
print(f"  Byte 7: {header[7]:02x} = {header[7]}")
print(f"  Bytes 8-14: {' '.join(f'{b:02x}' for b in header[8:15])}")

# Actually, let me reconsider. The 16-byte spacing starts at offset 15.
# But what if the FIRST record actually starts at offset 0 and is ALSO 16 bytes?
# Let's check: offset 0 = 00 01 0a 38 26 05 02 1a 01 00 04 00 00 04 02 d0
# And all subsequent records are 16 bytes. Let's see if the first is also a data record.

# The key insight: stride=16 with constant positions: 
# pos 0,2,3,4,6,7,8,9,10,12,13 have very few unique values
# That means positions 1,5,11,14,15 are the variable data fields

# Let's analyze each position across many records
print("\n=== Per-position analysis (16-byte records from offset 15, first 1000 records) ===")
n_recs = min(1000, (len(data) - 15) // 16)
for pos in range(16):
    vals = set()
    val_list = []
    for r in range(n_recs):
        v = data[15 + r*16 + pos]
        vals.add(v)
        val_list.append(v)
    if len(vals) <= 10:
        print(f"  pos {pos:2d}: {len(vals):3d} unique values: {sorted(vals)}")
    else:
        print(f"  pos {pos:2d}: {len(vals):3d} unique values, range [{min(val_list)}, {max(val_list)}], "
              f"mean={sum(val_list)/len(val_list):.1f}")

# Now let me look at the REV CAN protocol more carefully.
# SparkMax CAN frames: The arbitration ID encodes device type, manufacturer, API, and device number
# Format: [device_type:5][manufacturer:8][api_class:4][api_index:6][device_number:6]
# REV = manufacturer 5, device type 2 (motor controller)
# So the CAN ID = (2 << 24) | (5 << 16) | (api_class << 10) | (api_index << 6) | device_num
# = 0x02050000 | ...

# In our records, we see bytes 6-9 = "b8 05 02 00" consistently
# As u32 LE: 0x000205b8 = 132536
# As u32 BE: 0xb8050200 = 3087827456

# Let's look at byte 5 which varies (device_id?)
print("\n=== Byte 5 values (potential device/signal ID) ===")
byte5_vals = []
for r in range(n_recs):
    byte5_vals.append(data[15 + r*16 + 5])
from collections import Counter
byte5_freq = Counter(byte5_vals)
for val, count in byte5_freq.most_common(20):
    print(f"  0x{val:02x} ({val:3d}): {count:5d} ({100*count/n_recs:.1f}%)")

# Let's also look at combinations of multiple bytes
print("\n=== Byte 5-9 combinations (potential CAN IDs) ===")
can_id_freq = Counter()
for r in range(n_recs):
    off = 15 + r*16
    chunk = tuple(data[off+5:off+10])
    can_id_freq[chunk] += 1
for chunk, count in can_id_freq.most_common(20):
    hex_str = " ".join(f"{b:02x}" for b in chunk)
    # Interpret as CAN arbitration ID
    # Try bytes 5-8 as u32 LE
    can_u32 = chunk[0] | (chunk[1] << 8) | (chunk[2] << 16) | (chunk[3] << 24)
    # REV CAN ID: device_num = bits 0-5, api_index = bits 6-11, api_class = bits 12-15
    # manufacturer = bits 16-23, device_type = bits 24-28
    dev_num = can_u32 & 0x3F
    api_idx = (can_u32 >> 6) & 0x3F
    api_cls = (can_u32 >> 12) & 0xF
    mfg = (can_u32 >> 16) & 0xFF
    dev_type = (can_u32 >> 24) & 0x1F
    print(f"  {hex_str}: count={count:5d}  CAN_ID=0x{can_u32:08x}  "
          f"devType={dev_type} mfg={mfg} apiCls={api_cls} apiIdx={api_idx} devNum={dev_num}")

# Try different byte ranges for CAN ID
print("\n=== Byte 6-9 as CAN ID (u32 LE) ===")
can_id_freq2 = Counter()
for r in range(n_recs):
    off = 15 + r*16
    can_u32 = struct.unpack_from('<I', data, off+6)[0]
    can_id_freq2[can_u32] += 1
for can_u32, count in can_id_freq2.most_common(10):
    dev_num = can_u32 & 0x3F
    api_idx = (can_u32 >> 6) & 0x3F
    api_cls = (can_u32 >> 12) & 0xF
    mfg = (can_u32 >> 16) & 0xFF
    dev_type = (can_u32 >> 24) & 0x1F
    print(f"  0x{can_u32:08x}: count={count:5d}  devType={dev_type} mfg={mfg} apiCls={api_cls} apiIdx={api_idx} devNum={dev_num}")

# Let's also look at the "data" fields
# Positions 1, 11, 14, 15 are variable. But actually let me look at the FULL picture.
# Record at offset 15: d0 1e ef 15 00 00 36 b8 05 02 00 00 ee 06 00 18
# Let's see bytes 0-1 as u16 LE — could be the CAN data payload

print("\n=== First 30 records fully decoded ===")
print("  rec#  bytes[0:2]  bytes[2:6]   bytes[6:10]  bytes[10:14] bytes[14:16]")
for r in range(30):
    off = 15 + r*16
    rec = data[off:off+16]
    b01 = struct.unpack_from('<H', rec, 0)[0]
    b26 = struct.unpack_from('<I', rec, 2)[0]
    b610 = struct.unpack_from('<I', rec, 6)[0]
    b1014 = struct.unpack_from('<I', rec, 10)[0]
    b1416 = struct.unpack_from('<H', rec, 14)[0]
    print(f"  {r:4d}  {b01:5d} (0x{b01:04x})  {b26:10d}  0x{b610:08x}  0x{b1014:08x}  {b1416:5d} (0x{b1416:04x})")
