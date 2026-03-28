"""
Full analysis of the REV log file.

Format (discovered):
  - Bytes 0-14: Header (15 bytes)
    - 00 01: version (1)
    - 0a: header marker or record type 
    - 38 26 05 02: first timestamp (u32 LE in some unit)
    - remaining header bytes TBD
    
  - After header: 16-byte records
    Record layout (16 bytes each, starting at offset 15):
      [0:2]  u16 LE - payload/status bytes
      [2:6]  u32 LE - timestamp (monotonic counter, unit TBD)
      [6:10] u32 LE - CAN arbitration ID (REV format)
      [10:14] u32 LE - CAN data (first 4 bytes)
      [14:16] u16 LE - CAN data (last 2 bytes) or additional payload

REV CAN ID format (29-bit extended):
  [device_type:5][manufacturer:8][api_class:4][api_index:6][device_number:6]
  For SparkMax/Flex: device_type=2, manufacturer=5
  
SparkMax periodic status frames:
  API Class 6 (Status 0): Applied output, faults, sticky faults, etc.
  API Class 7 (Status 1): Velocity, temperature, voltage, current
  API Class 8 (Status 2): Position
  API Class 11 (0xB): Periodic status (combined)
"""
import struct
from collections import Counter, defaultdict

LOG_PATH = r"C:\Users\danla\Desktop\Logs2\$Idaho\REV_20241218_140212.revlog"

with open(LOG_PATH, "rb") as f:
    data = f.read()

HEADER_SIZE = 15
RECORD_SIZE = 16
n_records = (len(data) - HEADER_SIZE) // RECORD_SIZE

print(f"File size: {len(data):,} bytes")
print(f"Records: {n_records:,}")
print(f"Header: {' '.join(f'{b:02x}' for b in data[:HEADER_SIZE])}")

# Actually, let me reconsider the record layout. 
# Let me check if the first record is actually at offset 0 and is also 16 bytes
# Header byte pattern: 00 01 0a 38 26 05 02 1a 01 00 04 00 00 04 02 d0
# Next 16 bytes:        1e ef 15 00 00 36 b8 05 02 00 00 ee 06 00 18 a0
# Those "next 16" don't look like data records either. Let me check if the
# record boundary is elsewhere.

# Actually wait - the "b8 05 02" pattern spacing is exactly 16 from offset 22.
# offset 22 has b8 at position 22. 22 - 15 = 7 within the record.
# That means in each 16-byte record starting at offset 15:
#   position 7 = 0xb8 (mostly)
# But our CAN ID was bytes [6:10] = 0x0205b8XX
# Let me re-examine: CAN ID bytes within each record are at positions 6-9

# For the first "record" at offset 15: d0 1e ef 15 00 00 36 b8 05 02 00 00 ee 06 00 18
# bytes[6:10] = 36 b8 05 02 → u32 LE = 0x0205b836
# REV CAN: devType = (0x0205b836 >> 24) & 0x1F = 2
#          mfg = (0x0205b836 >> 16) & 0xFF = 5
#          apiCls = (0x0205b836 >> 12) & 0xF = 0xB = 11
#          apiIdx = (0x0205b836 >> 6) & 0x3F = (0xb836 >> 6) & 0x3F = 0x2E0 >> 6... wait

# Let me recalculate properly
# 0x0205b836:
# bit 28-24: (0x0205b836 >> 24) = 0x02 → device_type = 2 (motor controller)
# bit 23-16: (0x0205b836 >> 16) & 0xFF = 0x05 → manufacturer = 5 (REV)
# bit 15-12: (0x0205b836 >> 12) & 0xF = 0xB → api_class = 11
# bit 11-6:  (0x0205b836 >> 6) & 0x3F = (0xb836 >> 6) & 0x3F 
#            0xb836 = 47158, >> 6 = 736, & 0x3F = 736 & 63 = 736 % 64 = 32
#            api_index = 32
# bit 5-0:   0x0205b836 & 0x3F = 0x36 & 0x3F = 54 & 63 = 54
#            BUT wait, 0x36 = 54, and 54 decimal = device number

# So device numbers we see: 54, 55, 56, 61, 62
# These don't match typical FRC CAN IDs (usually 1-20ish)
# Wait - maybe the format is different. Let me check RobotMap for SparkMax IDs

print("\n=== Scanning ALL unique CAN IDs in file ===")
can_ids = Counter()
for r in range(n_records):
    off = HEADER_SIZE + r * RECORD_SIZE
    can_u32 = struct.unpack_from('<I', data, off + 6)[0]
    can_ids[can_u32] += 1

for can_id, count in can_ids.most_common(30):
    dev_num = can_id & 0x3F
    api_idx = (can_id >> 6) & 0x3F
    api_cls = (can_id >> 12) & 0xF
    mfg = (can_id >> 16) & 0xFF
    dev_type = (can_id >> 24) & 0x1F
    pct = 100 * count / n_records
    print(f"  0x{can_id:08x}: count={count:6d} ({pct:5.1f}%)  "
          f"dType={dev_type} mfg={mfg} aCls={api_cls} aIdx={api_idx} dNum={dev_num}")

# Timestamp analysis
print("\n=== Timestamp analysis ===")
ts_first = struct.unpack_from('<I', data, HEADER_SIZE + 2)[0]
ts_last = struct.unpack_from('<I', data, HEADER_SIZE + (n_records-1)*RECORD_SIZE + 2)[0]
print(f"  First timestamp: {ts_first}")
print(f"  Last timestamp:  {ts_last}")
print(f"  Range: {ts_last - ts_first}")
print(f"  If ms: {(ts_last - ts_first)/1000:.1f}s = {(ts_last - ts_first)/60000:.1f}min")
print(f"  If us: {(ts_last - ts_first)/1e6:.1f}s")

# Sample every 1000 records
print("\n=== Timestamp progression (every 10000 records) ===")
for r in range(0, n_records, max(1, n_records // 30)):
    off = HEADER_SIZE + r * RECORD_SIZE
    ts = struct.unpack_from('<I', data, off + 2)[0]
    can_id = struct.unpack_from('<I', data, off + 6)[0]
    dev_num = can_id & 0x3F
    payload1 = struct.unpack_from('<I', data, off + 10)[0]
    payload2 = struct.unpack_from('<H', data, off + 14)[0]
    print(f"  rec {r:7d}: ts={ts:10d}  CAN=0x{can_id:08x} dev#{dev_num:2d}  payload=0x{payload1:08x} 0x{payload2:04x}")

# Let me also try a different record start offset
# What if the header is not 15 bytes but something else?
# Let's check: are there any records that DON'T have devType=2, mfg=5?
print("\n=== Non-REV-motor CAN IDs ===")
for can_id, count in can_ids.most_common():
    dev_type = (can_id >> 24) & 0x1F
    mfg = (can_id >> 16) & 0xFF
    if not (dev_type == 2 and mfg == 5):
        print(f"  0x{can_id:08x}: count={count:6d}  dType={dev_type} mfg={mfg}")
    if count < 10:
        break

# Try DIFFERENT header size (maybe it's not 15)
print("\n=== Testing alternative header sizes ===")
for hdr_size in [0, 1, 2, 3, 4, 5, 10, 11, 12, 13, 14, 15, 16]:
    n_rec = (len(data) - hdr_size) // RECORD_SIZE
    if n_rec < 100:
        continue
    # Check how many records have valid REV CAN IDs
    valid = 0
    for r in range(min(500, n_rec)):
        off = hdr_size + r * RECORD_SIZE
        can_u32 = struct.unpack_from('<I', data, off + 6)[0]
        dev_type = (can_u32 >> 24) & 0x1F
        mfg = (can_u32 >> 16) & 0xFF
        if dev_type == 2 and mfg == 5:
            valid += 1
    print(f"  hdr={hdr_size:2d}: {valid}/500 records have REV CAN IDs ({100*valid/500:.0f}%)")
