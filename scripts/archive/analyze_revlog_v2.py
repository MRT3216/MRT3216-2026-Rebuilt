"""
MRT3216 — REV Hardware Client Log (.revlog) Analyzer v2
========================================================
Improved version that handles the mixed record-type format better.
Focuses on identifying contiguous sessions and filtering out misaligned records.

Key insight from v1: Only ~48K of ~664K records are REV motor frames.
The remaining ~616K are likely other record types (heartbeats, config, etc.)
or the file format is more complex (e.g., variable-length records).

Strategy: Focus on frames that form tight timing clusters (actual comm sessions)
and discard outlier timestamps that come from misaligned reads.
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
OUTPUT_PATH = os.path.join(SCRIPT_DIR, "revlog_analysis_v2.txt")

HEADER_SIZE = 15
RECORD_SIZE = 16

DEVICE_NAMES = {
    51: "Flywheel Left (Kraken)",
    52: "Flywheel Right (Kraken)",
    53: "Hood (Kraken)",
    54: "Turret (SparkMax NEO)",
    55: "Spindexer (SparkMax)",
    56: "Kicker (SparkMax)",
    60: "Intake Rollers (Kraken)",
    61: "Intake Pivot Left (SparkFlex)",
    62: "Intake Pivot Right (SparkFlex)",
}

_out = None


def p(*a, **kw):
    print(*a, **kw)
    if _out:
        print(*a, **kw, file=_out)
        _out.flush()


def section(title):
    p(f"\n{'='*72}")
    p(f"  {title}")
    p(f"{'='*72}")


def parse_can_id(can_u32):
    return {
        "device_type": (can_u32 >> 24) & 0x1F,
        "manufacturer": (can_u32 >> 16) & 0xFF,
        "api_class": (can_u32 >> 12) & 0xF,
        "api_index": (can_u32 >> 6) & 0x3F,
        "device_number": can_u32 & 0x3F,
    }


def is_rev_motor(can_u32):
    f = parse_can_id(can_u32)
    return f["device_type"] == 2 and f["manufacturer"] == 5


def find_sessions(timestamps_ms, max_gap_ms=5000):
    """Split a list of timestamps into contiguous sessions (max_gap_ms between frames)."""
    if not timestamps_ms:
        return []
    ts = sorted(timestamps_ms)
    sessions = []
    start = ts[0]
    prev = ts[0]
    count = 1
    for t in ts[1:]:
        if t - prev > max_gap_ms:
            sessions.append((start, prev, count))
            start = t
            count = 1
        else:
            count += 1
        prev = t
    sessions.append((start, prev, count))
    return sessions


def analyze():
    global _out
    _out = open(OUTPUT_PATH, "w", encoding="utf-8")

    p("MRT3216 — REV Hardware Client Log Analysis (v2)")
    p(f"Log file: {os.path.basename(LOG_PATH)}")
    fsize = os.path.getsize(LOG_PATH)
    p(f"File size: {fsize:,} bytes ({fsize/1024/1024:.2f} MB)")

    with open(LOG_PATH, "rb") as f:
        data = f.read()

    n_records = (len(data) - HEADER_SIZE) // RECORD_SIZE
    p(f"Total records (assuming 16-byte fixed): {n_records:,}")
    p(f"File date (from filename): 2024-12-18 14:02:12")

    # ── Parse all REV motor frames ──
    section("PARSING")
    frames = []  # (record_index, ts_ms, dev_num, api_class, api_index, payload_6bytes)
    non_rev = 0

    for r in range(n_records):
        off = HEADER_SIZE + r * RECORD_SIZE
        ts_ms = struct.unpack_from("<I", data, off + 2)[0]
        can_u32 = struct.unpack_from("<I", data, off + 6)[0]

        if not is_rev_motor(can_u32):
            non_rev += 1
            continue

        f = parse_can_id(can_u32)
        payload = data[off + 10 : off + 16]
        frames.append((r, ts_ms, f["device_number"], f["api_class"], f["api_index"], payload))

    p(f"  REV motor frames: {len(frames):,}")
    p(f"  Non-REV frames: {non_rev:,}")

    # ── Identify valid timestamp range ──
    # The REV Hardware Client timestamps should be wall-clock ms since some epoch.
    # From the filename: 2024-12-18 14:02:12
    # Let's look at the timestamp distribution to find the real session.
    all_ts = [f[1] for f in frames]
    ts_counter = Counter()
    for ts in all_ts:
        # Bucket into 10-second bins
        ts_counter[ts // 10000] += 1

    # Find the densest time region — that's the real session
    # Sort buckets by count, take top ones
    top_buckets = ts_counter.most_common(50)
    p(f"\n  Top 20 10s-buckets by frame count:")
    for bucket, cnt in top_buckets[:20]:
        t_sec = bucket * 10
        p(f"    t={t_sec:>12,}s ({t_sec/3600:.1f}h): {cnt} frames")

    # The real session timestamps should cluster tightly.
    # Let's find the primary cluster by looking at timestamp distribution.
    section("SESSION DETECTION")

    # Group frames by device
    device_frames = defaultdict(list)
    for rec_idx, ts, dev, api_cls, api_idx, payload in frames:
        device_frames[dev].append((ts, api_cls, api_idx, payload))

    for dev in sorted(device_frames.keys()):
        name = DEVICE_NAMES.get(dev, f"Unknown #{dev}")
        dev_ts = [f[0] for f in device_frames[dev]]
        sessions = find_sessions(dev_ts, max_gap_ms=10000)

        if len(device_frames[dev]) < 5:
            p(f"\n  Device {dev} ({name}): {len(device_frames[dev])} frames (too few)")
            continue

        p(f"\n  Device {dev} ({name}): {len(device_frames[dev])} frames, {len(sessions)} session(s)")
        for i, (s_start, s_end, s_count) in enumerate(sessions):
            dur = (s_end - s_start) / 1000
            rate = s_count / max(dur, 0.001)
            p(f"    Session {i+1}: {s_count:>6,} frames, "
              f"t=[{s_start:>12,} → {s_end:>12,}] ms, "
              f"dur={dur:>7.1f}s ({dur/60:.1f}min), "
              f"rate={rate:.1f} Hz")

    # ── Focus on the primary session for each device ──
    section("PRIMARY SESSION ANALYSIS")
    primary_data = {}  # dev -> list of (ts, api_cls, api_idx, payload)

    for dev in sorted(device_frames.keys()):
        dev_ts = [f[0] for f in device_frames[dev]]
        sessions = find_sessions(dev_ts, max_gap_ms=10000)
        if not sessions:
            continue
        # Pick the session with the most frames
        best_session = max(sessions, key=lambda s: s[2])
        s_start, s_end, s_count = best_session
        # Filter frames to this session
        primary = [(ts, ac, ai, pl) for (ts, ac, ai, pl) in device_frames[dev]
                   if s_start <= ts <= s_end]
        primary_data[dev] = primary

    for dev in sorted(primary_data.keys()):
        name = DEVICE_NAMES.get(dev, f"Unknown #{dev}")
        pframes = primary_data[dev]
        if len(pframes) < 10:
            continue

        ts_list = [f[0] for f in pframes]
        dur_s = (max(ts_list) - min(ts_list)) / 1000
        p(f"\n  Device {dev} ({name})")
        p(f"    Frames in primary session: {len(pframes)}")
        p(f"    Duration: {dur_s:.1f}s ({dur_s/60:.1f} min)")

        # API class breakdown
        api_counts = Counter(f[1] for f in pframes)
        p(f"    API classes: {dict(api_counts.most_common())}")

        # Frame timing
        ts_sorted = sorted(ts_list)
        deltas = [ts_sorted[i+1] - ts_sorted[i] for i in range(len(ts_sorted)-1)]
        deltas = [d for d in deltas if d > 0]
        if deltas:
            p(f"    Inter-frame timing: "
              f"min={min(deltas)}ms, median={statistics.median(deltas):.0f}ms, "
              f"mean={statistics.mean(deltas):.1f}ms, max={max(deltas)}ms")
            gaps = [d for d in deltas if d > 1000]
            if gaps:
                p(f"    Gaps >1s: {len(gaps)} (max {max(gaps)/1000:.1f}s)")

        # Payload analysis per API class
        for api_cls in sorted(api_counts.keys()):
            cls_frames = [(ts, pl) for (ts, ac, ai, pl) in pframes if ac == api_cls]
            if len(cls_frames) < 5:
                continue

            p(f"\n    API class {api_cls} ({len(cls_frames)} frames):")

            # Check if payload is constant (likely config/ID frame)
            unique_payloads = len(set(pl.hex() for _, pl in cls_frames))
            if unique_payloads == 1:
                pl = cls_frames[0][1]
                p(f"      Constant payload: {pl.hex()}")
                # Decode as potential firmware version / device info
                v1 = struct.unpack_from("<H", pl, 0)[0]
                v2 = struct.unpack_from("<H", pl, 2)[0]
                v3 = struct.unpack_from("<H", pl, 4)[0]
                p(f"      As u16s: [{v1}, {v2}, {v3}]")
                continue

            # Variable payload — extract statistics
            for byte_pos in range(0, 6, 2):
                vals = []
                for _, pl in cls_frames:
                    if byte_pos + 2 <= len(pl):
                        vals.append(struct.unpack_from("<h", pl, byte_pos)[0])
                if vals and (max(vals) != min(vals)):
                    p(f"      i16[{byte_pos}:{byte_pos+2}]: "
                      f"min={min(vals):>7d}  max={max(vals):>7d}  "
                      f"mean={statistics.mean(vals):>8.1f}  "
                      f"σ={statistics.stdev(vals):>8.1f}")

            # Unsigned byte analysis for likely temperature (single byte, moderate range)
            for bp in range(6):
                vals = [pl[bp] for _, pl in cls_frames]
                u_vals = set(vals)
                if len(u_vals) > 1 and len(u_vals) < 50 and min(vals) >= 15 and max(vals) <= 80:
                    p(f"      byte[{bp}] (possible temp): "
                      f"min={min(vals)} max={max(vals)} mean={statistics.mean(vals):.1f}")

    # ── Cross-reference ──
    section("DEVICE SUMMARY")
    p("  REV Hardware Client logs only capture REV motor controllers (SparkMax/SparkFlex).")
    p("  CTRE/Phoenix motors (TalonFX/Kraken) are NOT captured — use .wpilog for those.\n")

    p(f"  {'CAN':>3s}  {'Subsystem':<35s}  {'Bus':>4s}  {'Status':<20s}  {'Frames':>8s}")
    p(f"  {'---':>3s}  {'---':<35s}  {'---':>4s}  {'---':<20s}  {'---':>8s}")
    for dev_id in sorted(set(list(DEVICE_NAMES.keys()) + list(device_frames.keys()))):
        name = DEVICE_NAMES.get(dev_id, f"Unknown #{dev_id}")
        is_ctre = "Kraken" in name
        bus = "CTRE" if is_ctre else "REV"
        if dev_id in primary_data and len(primary_data[dev_id]) > 5:
            n = len(primary_data[dev_id])
            status = "✓ Logged"
        elif dev_id in device_frames and len(device_frames[dev_id]) > 0:
            n = len(device_frames[dev_id])
            status = "~ Sparse"
        else:
            n = 0
            status = "✗ Not in log" if not is_ctre else "— N/A (CTRE)"
        p(f"  {dev_id:>3d}  {name:<35s}  {bus:>4s}  {status:<20s}  {n:>8,}")

    # Unknown devices
    unknowns = [d for d in device_frames if d not in DEVICE_NAMES and len(device_frames[d]) > 2]
    if unknowns:
        p(f"\n  Unknown devices with >2 frames: {sorted(unknowns)}")
        for d in sorted(unknowns):
            p(f"    #{d}: {len(device_frames[d])} frames — "
              f"possibly from bench test or different robot config")

    # ── Non-REV frame analysis ──
    section("NON-REV FRAME SAMPLING")
    p("  Sampling first 20 non-REV frames to understand other record types:\n")
    sample_count = 0
    for r in range(min(n_records, 50000)):
        off = HEADER_SIZE + r * RECORD_SIZE
        ts_ms = struct.unpack_from("<I", data, off + 2)[0]
        can_u32 = struct.unpack_from("<I", data, off + 6)[0]
        if is_rev_motor(can_u32):
            continue
        raw = data[off : off + RECORD_SIZE]
        f = parse_can_id(can_u32)
        p(f"  rec {r:>6d}: {raw.hex()} | ts={ts_ms:>12,} "
          f"CAN={can_u32:08x} devType={f['device_type']} mfg={f['manufacturer']} "
          f"dev={f['device_number']} api={f['api_class']}.{f['api_index']}")
        sample_count += 1
        if sample_count >= 20:
            break

    # ── Conclusions ──
    section("CONCLUSIONS")
    p("  1. This is a REV Hardware Client log from 2024-12-18 (pre-season / bench test).")
    p("")
    real_devices = [d for d in primary_data if len(primary_data[d]) > 10 and d in DEVICE_NAMES]
    p(f"  2. {len(real_devices)} robot REV motor controllers were actively communicating:")
    for d in sorted(real_devices):
        pf = primary_data[d]
        dur = (max(f[0] for f in pf) - min(f[0] for f in pf)) / 1000
        p(f"     • CAN {d}: {DEVICE_NAMES[d]} — {len(pf)} frames over {dur:.0f}s")
    p("")
    missing_rev = [d for d in DEVICE_NAMES if "Kraken" not in DEVICE_NAMES[d] and d not in real_devices]
    if missing_rev:
        p(f"  3. REV motors NOT found / very sparse:")
        for d in sorted(missing_rev):
            p(f"     • CAN {d}: {DEVICE_NAMES[d]}")
    p("")
    p("  4. ~92% of the raw 16-byte records did NOT match the REV CAN frame pattern,")
    p("     suggesting the .revlog format contains multiple record types (heartbeats,")
    p("     device config, firmware data, etc.) intermixed with CAN traffic data.")
    p("     Without the official REV format spec, these remain opaque.")
    p("")
    p("  5. The sessions appear short (~10 minutes each), consistent with bench testing")
    p("     or REV Hardware Client connection for firmware updates/configuration.")
    p("")
    p("  6. API class 2 payloads are constant per device — likely device identity or")
    p("     firmware version frames. API class 11 (0xB) contains variable data —")
    p("     likely periodic status telemetry (velocity, temperature, voltage, current).")

    p(f"\n  Output written to: {OUTPUT_PATH}")
    _out.close()


if __name__ == "__main__":
    analyze()
