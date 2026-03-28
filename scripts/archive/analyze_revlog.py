"""
MRT3216 — REV Hardware Client Log (.revlog) Analyzer
=====================================================
Decodes raw CAN frames logged by the REV Hardware Client from SparkMax/SparkFlex
motor controllers.  Produces a text report summarising motor behaviour per device.

Binary format (reverse-engineered):
  Header  : 15 bytes
    [0:2]  version (0x0001 LE)
    [2]    marker (0x0a)
    [3:7]  first timestamp (u32 LE, unit = ms)
    [7:15] additional header (device info / config)

  Records : 16 bytes each, starting at offset 15
    [0:2]  payload prefix (u16 LE) — low-level CAN frame metadata / padding
    [2:6]  timestamp_ms (u32 LE) — monotonic milliseconds
    [6:10] CAN arbitration ID (u32 LE, 29-bit REV extended ID)
    [10:14] CAN data bytes 0-3 (u32 LE)
    [14:16] CAN data bytes 4-5 (u16 LE)

REV CAN arbitration-ID layout (29-bit):
  [28:24] device_type   (2 = motor controller)
  [23:16] manufacturer  (5 = REV)
  [15:12] api_class
  [11:6]  api_index
  [5:0]   device_number (= CAN ID)

SparkMax/Flex periodic status frames:
  API class 1, index 0-3 : Status 0  (applied output %, faults)
  API class 1, index 4-7 : Status 1  (velocity, temperature, supply voltage, current)
  API class 1, index 8-11: Status 2  (position)
  API class 2             : Status 3  (analog sensor)
  API class 3             : Status 4  (alternate encoder)

In practice, REV Hardware Client logs primarily periodic status 0, 1, 2 frames.

Status 1 payload (8 bytes):
  [0:2] velocity (i16, RPM * factor)
  [2]   temperature (u8, °C)
  [3:5] supply voltage (u16, mV * 4 or similar scale)
  [5:7] current (u16, mA * factor)

This script attempts heuristic extraction of telemetry values from the raw CAN
data even without a complete REV CAN protocol specification.

Usage:
    .venv/Scripts/python.exe scripts/analyze_revlog.py
"""

import struct
import os
import sys
from collections import Counter, defaultdict
import statistics

# ── Configuration ───────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_PATH = os.path.join(
    os.path.expanduser("~/Desktop/Logs2/$Idaho"),
    "REV_20241218_140212.revlog",
)
OUTPUT_PATH = os.path.join(SCRIPT_DIR, "revlog_analysis.txt")

HEADER_SIZE = 15
RECORD_SIZE = 16

# Map CAN device numbers to robot subsystem names (from RobotMap.java)
DEVICE_NAMES = {
    51: "Flywheel Left (Kraken)",
    52: "Flywheel Right (Kraken)",
    53: "Hood (Kraken)",
    54: "Turret (SparkMax NEO)",
    55: "Spindexer",
    56: "Kicker",
    60: "Intake Rollers (Kraken)",
    61: "Intake Pivot Left (SparkFlex)",
    62: "Intake Pivot Right (SparkFlex)",
}

# ── Output helper ────────────────────────────────────────────────────────────
_out = None


def p(*args, **kwargs):
    """Print to both stdout and output file."""
    print(*args, **kwargs)
    if _out:
        print(*args, **kwargs, file=_out)
        _out.flush()


def section(title):
    p(f"\n{'='*72}")
    p(f"  {title}")
    p(f"{'='*72}")


# ── CAN ID parsing ──────────────────────────────────────────────────────────
def parse_can_id(can_u32):
    """Extract REV CAN ID fields."""
    return {
        "device_type": (can_u32 >> 24) & 0x1F,
        "manufacturer": (can_u32 >> 16) & 0xFF,
        "api_class": (can_u32 >> 12) & 0xF,
        "api_index": (can_u32 >> 6) & 0x3F,
        "device_number": can_u32 & 0x3F,
    }


def is_rev_motor(can_u32):
    fields = parse_can_id(can_u32)
    return fields["device_type"] == 2 and fields["manufacturer"] == 5


# ── Main analysis ────────────────────────────────────────────────────────────
def analyze():
    global _out
    _out = open(OUTPUT_PATH, "w", encoding="utf-8")

    p("MRT3216 — REV Hardware Client Log Analysis")
    p(f"Log file: {LOG_PATH}")
    p(f"File size: {os.path.getsize(LOG_PATH):,} bytes")

    with open(LOG_PATH, "rb") as f:
        data = f.read()

    n_records = (len(data) - HEADER_SIZE) // RECORD_SIZE
    p(f"Total records: {n_records:,}")

    # ── Header ──
    section("HEADER")
    hdr = data[:HEADER_SIZE]
    p(f"  Raw: {' '.join(f'{b:02x}' for b in hdr)}")
    version = struct.unpack_from("<H", hdr, 0)[0]
    first_ts = struct.unpack_from("<I", hdr, 3)[0]
    p(f"  Version: {version}")
    p(f"  First timestamp: {first_ts} ms ({first_ts/1000:.1f}s)")
    p(f"  File date (from name): 2024-12-18 14:02:12")

    # ── Parse all records ──
    section("PARSING RECORDS")
    # Per-device data collection
    device_timestamps = defaultdict(list)  # dev_num -> [ts_ms, ...]
    device_can_ids = defaultdict(Counter)  # dev_num -> {can_id: count}
    device_api_classes = defaultdict(Counter)  # dev_num -> {api_class: count}
    device_payloads = defaultdict(list)  # (dev_num, api_class) -> [(ts, data6), ...]

    non_rev_count = 0
    rev_count = 0

    for r in range(n_records):
        off = HEADER_SIZE + r * RECORD_SIZE
        ts_ms = struct.unpack_from("<I", data, off + 2)[0]
        can_u32 = struct.unpack_from("<I", data, off + 6)[0]
        payload = data[off + 10 : off + 16]  # 6 bytes of CAN data

        if not is_rev_motor(can_u32):
            non_rev_count += 1
            continue

        rev_count += 1
        fields = parse_can_id(can_u32)
        dev = fields["device_number"]
        api_cls = fields["api_class"]

        device_timestamps[dev].append(ts_ms)
        device_can_ids[dev][can_u32] += 1
        device_api_classes[dev][api_cls] += 1
        device_payloads[(dev, api_cls)].append((ts_ms, payload))

    p(f"  REV motor controller frames: {rev_count:,}")
    p(f"  Non-REV / unrecognized frames: {non_rev_count:,}")
    p(f"  Unique devices: {sorted(device_timestamps.keys())}")

    # ── Per-device summary ──
    section("DEVICE OVERVIEW")
    for dev in sorted(device_timestamps.keys()):
        name = DEVICE_NAMES.get(dev, f"Unknown Device #{dev}")
        ts_list = device_timestamps[dev]
        duration_s = (max(ts_list) - min(ts_list)) / 1000.0 if ts_list else 0
        p(f"\n  Device {dev}: {name}")
        p(f"    Total frames: {len(ts_list):,}")
        p(f"    Time span: {duration_s:.1f}s ({duration_s/60:.1f}min)")
        p(f"    Avg frame rate: {len(ts_list)/max(duration_s,0.001):.1f} frames/s")
        p(f"    API classes: {dict(device_api_classes[dev].most_common())}")

    # ── Time range ──
    section("TIMELINE")
    all_ts = []
    for ts_list in device_timestamps.values():
        all_ts.extend(ts_list)
    if all_ts:
        t_min = min(all_ts)
        t_max = max(all_ts)
        p(f"  First record: {t_min} ms")
        p(f"  Last record:  {t_max} ms")
        p(f"  Total span: {(t_max - t_min)/1000:.1f}s = {(t_max - t_min)/60000:.1f}min")

    # ── Heuristic telemetry extraction ──
    # The REV CAN protocol encodes Status 1 data as:
    #   The api_class for periodic status frames appears to be 11 (0xB) based on our data
    #   Let's try to extract meaningful data by looking at the numeric ranges of payload bytes
    section("PAYLOAD ANALYSIS (per device, per API class)")
    for dev in sorted(device_timestamps.keys()):
        name = DEVICE_NAMES.get(dev, f"Unknown #{dev}")
        for api_cls in sorted(device_api_classes[dev].keys()):
            frames = device_payloads.get((dev, api_cls), [])
            if len(frames) < 10:
                continue

            p(f"\n  Device {dev} ({name}), API class {api_cls}: {len(frames)} frames")

            # Analyze each pair of bytes as i16 LE
            for byte_pos in range(0, 6, 2):
                vals = []
                for ts, payload in frames:
                    if byte_pos + 2 <= len(payload):
                        v = struct.unpack_from("<h", payload, byte_pos)[0]
                        vals.append(v)
                if vals:
                    p(
                        f"    bytes[{byte_pos}:{byte_pos+2}] as i16: "
                        f"min={min(vals):6d}  max={max(vals):6d}  "
                        f"mean={statistics.mean(vals):8.1f}  "
                        f"stdev={statistics.stdev(vals) if len(vals)>1 else 0:.1f}"
                    )

            # Also analyze individual bytes (useful for temperature which is u8)
            for byte_pos in range(6):
                vals = [payload[byte_pos] for _, payload in frames]
                if vals and max(vals) != min(vals):
                    p(
                        f"    byte[{byte_pos}] as u8:  "
                        f"min={min(vals):3d}  max={max(vals):3d}  "
                        f"mean={statistics.mean(vals):6.1f}  "
                        f"unique={len(set(vals))}"
                    )

    # ── Activity timeline (10-second buckets) ──
    section("ACTIVITY TIMELINE (10s buckets)")
    if all_ts:
        bucket_size_ms = 10000
        t_base = min(all_ts)
        n_buckets = ((max(all_ts) - t_base) // bucket_size_ms) + 1
        if n_buckets > 300:
            bucket_size_ms = 60000  # fall back to 1-minute buckets
            n_buckets = ((max(all_ts) - t_base) // bucket_size_ms) + 1
            p(f"  (Using 60s buckets, {n_buckets} total)")
        else:
            p(f"  (Using 10s buckets, {n_buckets} total)")

        for dev in sorted(device_timestamps.keys()):
            buckets = [0] * n_buckets
            for ts in device_timestamps[dev]:
                b = (ts - t_base) // bucket_size_ms
                if 0 <= b < n_buckets:
                    buckets[b] += 1
            name = DEVICE_NAMES.get(dev, f"Dev#{dev}")
            active_buckets = sum(1 for c in buckets if c > 0)
            total_buckets = len(buckets)
            p(f"  {name:>35s} (#{dev}): active {active_buckets}/{total_buckets} buckets "
              f"({100*active_buckets/max(total_buckets,1):.0f}%), "
              f"peak={max(buckets)} frames/bucket")

    # ── Cross-reference with wpilog devices ──
    section("CROSS-REFERENCE WITH ROBOT CAN MAP")
    p("  CAN ID → Subsystem mapping from RobotMap.java:")
    for dev_id, dev_name in sorted(DEVICE_NAMES.items()):
        found = dev_id in device_timestamps
        status = "✓ FOUND" if found else "✗ NOT IN LOG"
        if found:
            count = len(device_timestamps[dev_id])
            p(f"    CAN {dev_id:2d}: {dev_name:40s} {status} ({count:,} frames)")
        else:
            p(f"    CAN {dev_id:2d}: {dev_name:40s} {status}")

    # Report unknown devices
    unknown = [d for d in device_timestamps if d not in DEVICE_NAMES]
    if unknown:
        p("\n  Unknown devices in log (not in RobotMap):")
        for dev in sorted(unknown):
            count = len(device_timestamps[dev])
            p(f"    CAN {dev:2d}: {count:,} frames")

    # ── Timing analysis per device ──
    section("FRAME TIMING ANALYSIS")
    for dev in sorted(device_timestamps.keys()):
        name = DEVICE_NAMES.get(dev, f"Unknown #{dev}")
        ts_list = sorted(device_timestamps[dev])
        if len(ts_list) < 2:
            continue
        deltas = [ts_list[i+1] - ts_list[i] for i in range(len(ts_list)-1)]
        deltas = [d for d in deltas if d > 0]  # filter zero-delta duplicates
        if not deltas:
            continue
        p(f"\n  Device {dev} ({name}): {len(deltas)} intervals")
        p(f"    Min delta:  {min(deltas):6d} ms")
        p(f"    Max delta:  {max(deltas):6d} ms")
        p(f"    Mean delta: {statistics.mean(deltas):8.1f} ms")
        p(f"    Median:     {statistics.median(deltas):8.1f} ms")
        if len(deltas) > 10:
            sorted_d = sorted(deltas)
            p95 = sorted_d[int(0.95 * len(sorted_d))]
            p(f"    p95 delta:  {p95:6d} ms")
        # Gaps > 1 second
        gaps = [d for d in deltas if d > 1000]
        if gaps:
            p(f"    Gaps >1s:   {len(gaps)} (max gap: {max(gaps)/1000:.1f}s)")

    # ── Summary ──
    section("SUMMARY")
    p(f"  File: {os.path.basename(LOG_PATH)}")
    p(f"  Date: 2024-12-18 (from filename)")
    p(f"  Total frames: {n_records:,} ({rev_count:,} REV motor, {non_rev_count:,} other)")
    if all_ts:
        p(f"  Duration: {(max(all_ts)-min(all_ts))/60000:.1f} minutes")
    p(f"  Devices found: {len(device_timestamps)}")
    for dev in sorted(device_timestamps.keys()):
        name = DEVICE_NAMES.get(dev, f"Unknown #{dev}")
        p(f"    #{dev}: {name}")

    # Conclusions
    known_devs = set(device_timestamps.keys()) & set(DEVICE_NAMES.keys())
    missing_devs = set(DEVICE_NAMES.keys()) - set(device_timestamps.keys())
    p(f"\n  Known robot devices logged: {len(known_devs)}/{len(DEVICE_NAMES)}")
    if missing_devs:
        p(f"  Missing from log: {sorted(missing_devs)}")
        for d in sorted(missing_devs):
            p(f"    CAN {d}: {DEVICE_NAMES[d]}")

    p(f"\n  NOTE: This is a REV Hardware Client log. It only captures SparkMax/SparkFlex")
    p(f"  devices. TalonFX/Kraken motors (Flywheel, Hood, Kicker, Intake Rollers) use")
    p(f"  Phoenix 6 / CTRE and will NOT appear in REV logs. They appear in the .wpilog")
    p(f"  via AdvantageKit.")

    p(f"\n  Output written to: {OUTPUT_PATH}")
    _out.close()


if __name__ == "__main__":
    analyze()
