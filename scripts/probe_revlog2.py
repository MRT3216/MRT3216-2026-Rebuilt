"""
Deeper probe of the REV log format. The pattern 'b8 05 02' repeats heavily.
Let's figure out the record structure.
"""
import struct

LOG_PATH = r"C:\Users\danla\Desktop\Logs2\$Idaho\REV_20241218_140212.revlog"

with open(LOG_PATH, "rb") as f:
    data = f.read()

print(f"File size: {len(data):,} bytes")

# The header appears to be: 00 01 0a 38 26 05 02 1a 01 00 04 00 00 04 02 d0
# After that, records seem to start. Let's look at the repeating pattern.

# Find all occurrences of 'b8 05 02' in first 1KB
pattern = bytes([0xb8, 0x05, 0x02])
print("\n=== Occurrences of b8 05 02 in first 1KB ===")
positions = []
for i in range(min(1024, len(data))):
    if data[i:i+3] == pattern:
        positions.append(i)
        
for p in positions[:30]:
    context = data[max(0,p-5):p+13]
    hex_ctx = " ".join(f"{b:02x}" for b in context)
    print(f"  offset {p:4d} (0x{p:04x}): ... {hex_ctx}")

if len(positions) >= 2:
    diffs = [positions[i+1] - positions[i] for i in range(len(positions)-1)]
    print(f"\n  Spacings between occurrences: {diffs[:20]}")
    print(f"  Most common spacing: {max(set(diffs), key=diffs.count)}")

# Let's look at the header more carefully
# Bytes: 00 01 | 0a 38 26 05 02 | 1a | 01 00 04 00 00 04 02 d0
# 
# What if:
# Byte 0-1: version (0x0001)  
# Byte 2: record type (0x0a = 10 = header/descriptor?)
# Then after header, each record is fixed-size with a type byte

print("\n=== Analyzing record structure starting from byte 2 ===")
# Let's see if byte 2 (0x0a) is meaningful, then look at the pattern after the header

# After the first record (header?), let's find the next different pattern
# The header might end and data records begin. Let's check byte patterns.

# Skip past what looks like a header and see the repeating records
# From hex dump: after offset 0x0f (byte 15), we see:
#   1e ef 15 00 00 36 b8 05 02 00 00 ee 06 00 18 a0
# Let's try this as record format: maybe each record is 16 bytes?

print("\n=== Trying 16-byte records starting at offset 15 ===")
for rec_idx in range(10):
    off = 15 + rec_idx * 16
    if off + 16 > len(data):
        break
    rec = data[off:off+16]
    hex_str = " ".join(f"{b:02x}" for b in rec)
    # Try various interpretations
    # As: [u8, u16, u16, u8, u24, u16, u8, u8, u16, u8]
    # Or: [u16_ts, u16, u16, ...] 
    vals_u16 = [struct.unpack_from('<H', rec, i)[0] for i in range(0, 16, 2)]
    vals_u32 = [struct.unpack_from('<I', rec, i)[0] for i in range(0, 16, 4)]
    print(f"  rec[{rec_idx:2d}] @{off:4d}: {hex_str}")
    print(f"         u16: {vals_u16}")

# Actually, let me re-examine. The header is:
# 00 01 | 0a | 38 26 05 02 | 1a | 01 00 04 00 00 04 02 d0
# What if 00 01 = version, 0a = header size or record count, 
# 38 26 05 02 = timestamp (u32 LE = 33826360)?

print("\n=== Trying different record boundaries ===")
# Let's look for a fixed stride. From the data:
# offset 0x10 (16): 1e ef 15 00 00 | 36 b8 05 02 00 00 | ee 06 00 18 a0 00
# offset 0x20 (32): ef 15 00 00 37 | b8 05 02 00 00 | ed 06 00 18 b0 00 ef
# Hmm, that doesn't line up well with 16.

# Let me try stride 15 or 11 
# Actually look at the second-most-common patterns:
# "ef 15 00 00" appears, "b8 05 02 00 00" appears
# Maybe records are variable-length? Let's check if there are length bytes.

# Another approach: look at byte 0x05 (=5) which appears 6.4% - maybe it's a 
# device ID or signal count

# Let me try a completely different approach: treat the file as a sequence of
# frames where each frame has: [timestamp_u16] [data...]
# The 'b8 05' could be a CAN ID or device type

# Let's count total occurrences of b8 05 02 in the full file
count = 0
off = 0
while True:
    idx = data.find(pattern, off)
    if idx == -1:
        break
    count += 1
    off = idx + 1
print(f"\n  Total 'b8 05 02' occurrences in file: {count}")
print(f"  If each is a record, avg record size: {len(data)/count:.1f} bytes")

# Try looking for the REV Hardware Client's CAN frame log format
# SparkMax CAN IDs are typically 0x02050000 + device_id
# The b8 05 02 pattern could be part of CAN arbitration IDs

# Let's check: 0x0205 could be a REV manufacturer ID + device type
# And the bytes before might be the device-specific CAN ID component

# Try treating bytes as: [timestamp?] [can_id_stuff] [payload]
print("\n=== Interpreting as CAN-frame log ===")
# Let's see if the header defines signal mappings
# Byte 0-1: 00 01 (version 1?)  
# Byte 2: 0a (10 = number of signals? or header length?)
# Then maybe a signal table follows?

# Actually, let me look for the CAN device ID (we know turret is SparkMax ID)
# RobotMap has CAN IDs. The SparkMax turret is likely a specific CAN ID.
# REV CAN protocol: API ID format = [device_type:5][mfg:8][api_class:4][api_index:6][device_id:6]

# Let's look for common byte sequences at regular intervals
print("\n=== Byte at offset mod N analysis ===")
for stride in [11, 12, 13, 14, 15, 16, 17, 18]:
    # Check how consistent byte values are at position 0, 1, etc within stride
    n_recs = min(100, len(data) // stride)
    if n_recs < 10:
        continue
    consistency = []
    for pos_in_rec in range(stride):
        vals = set()
        for r in range(n_recs):
            vals.add(data[r * stride + pos_in_rec])
        consistency.append(len(vals))
    # A good stride will have some positions with very few unique values (constant fields)
    min_unique = min(consistency)
    const_positions = sum(1 for c in consistency if c <= 3)
    if const_positions >= 2:
        print(f"  stride={stride}: const-ish positions({const_positions}): {[(i,consistency[i]) for i in range(stride) if consistency[i] <= 3]}")

# New approach: the REV log might start with a descriptor record
# Let's read from right after 00 01 (version)
print("\n=== Parsing from offset 2 ===")
off = 2
# byte 2 = 0x0a (10). Maybe it's a record type.
rec_type = data[2]
print(f"  Record type at offset 2: 0x{rec_type:02x} = {rec_type}")

# After that: 38 26 05 02 = 33826360 as u32 LE — could be a timestamp in ms
ts = struct.unpack_from('<I', data, 3)[0]
print(f"  Possible timestamp at offset 3: {ts} (u32 LE)")
print(f"    As seconds: {ts/1000:.1f}s  As minutes: {ts/60000:.1f}min  As hours: {ts/3600000:.1f}h")

# What about as u24? 
ts24 = data[3] | (data[4] << 8) | (data[5] << 16)
print(f"  As u24 at offset 3: {ts24}")
print(f"    As seconds: {ts24/1000:.1f}s")

# Hmm, 33826360 ms = ~9.4 hours. That could be time-of-day from midnight
# 9.4 hours = roughly 9:24 AM. The file is named REV_20241218_140212 
# which suggests Dec 18 2024 at 2:02:12 PM
# 14:02 = 50532 seconds from midnight = 50532000 ms
# 33826360 ms = 33826.36 seconds = 9h 23m 46s — doesn't match 2:02 PM

# Try big-endian
ts_be = struct.unpack_from('>I', data, 3)[0]
print(f"  Possible timestamp at offset 3 (BE): {ts_be}")
print(f"    As ms→hours: {ts_be/3600000:.1f}h")

# Let's try the REV log format from the REV Hardware Client source
# It may use a simple format: each record = [u32 timestamp_ms] [u8 signal_id] [value bytes]
# Or it could be MessagePack

# Let's check if it's MessagePack
print("\n=== Checking for MessagePack ===")
try:
    import msgpack
    unpacker = msgpack.Unpacker(raw=False)
    unpacker.feed(data[:1024])
    for i, obj in enumerate(unpacker):
        print(f"  MsgPack obj[{i}]: {obj}")
        if i >= 5:
            break
except ImportError:
    print("  msgpack not installed")
except Exception as e:
    print(f"  Not MessagePack: {e}")

# Check for protobuf varint encoding
print("\n=== Check varint at offset 0 ===")
def read_varint(data, offset):
    result = 0
    shift = 0
    while offset < len(data):
        b = data[offset]
        result |= (b & 0x7F) << shift
        offset += 1
        if not (b & 0x80):
            return result, offset
        shift += 7
    return None, offset

val, next_off = read_varint(data, 0)
print(f"  Varint at 0: value={val}, next_offset={next_off}")
val, next_off = read_varint(data, next_off)
print(f"  Varint at {next_off-1}: value={val}, next_offset={next_off}")
