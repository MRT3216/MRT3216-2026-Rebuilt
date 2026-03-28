#!/usr/bin/env python3
"""Analyze Q61 for auto-period stuttering: loop overruns, GC, timing."""
import struct
from collections import defaultdict
from wpiutil.log import DataLogReader

LOG = r"C:\Users\danla\Desktop\Logs Idaho\akit_26-03-28_16-55-43_idbo_q61.wpilog"
print("Reading Q61 log...")

reader = DataLogReader(LOG)
entries = {}
for record in reader:
    if record.isStart():
        d = record.getStartData()
        entries[d.entry] = (d.name, d.type)

# Build entry ID lookups
TIMING_ENTRIES = {}
MODE_ENTRIES = {}
GC_ENTRIES = {}
QUEUED_ENTRIES = {}

for eid, (name, typ) in entries.items():
    if name == "/RealOutputs/LoggedRobot/FullCycleMS":
        TIMING_ENTRIES[eid] = "cycle_ms"
    elif name == "/RealOutputs/LoggedRobot/UserCodeMS":
        TIMING_ENTRIES[eid] = "user_ms"
    elif name == "/RealOutputs/LoggedRobot/LogPeriodicMS":
        TIMING_ENTRIES[eid] = "log_ms"
    elif name == "/RealOutputs/LoggedRobot/GCTimeMS":
        GC_ENTRIES[eid] = "gc_time"
    elif name == "/RealOutputs/LoggedRobot/GCCounts":
        GC_ENTRIES[eid] = "gc_count"
    elif name == "/RealOutputs/Logger/QueuedCycles":
        QUEUED_ENTRIES[eid] = "queued"
    elif name == "/DriverStation/Enabled":
        MODE_ENTRIES[eid] = "enabled"
    elif name == "/DriverStation/Autonomous":
        MODE_ENTRIES[eid] = "auto"
    elif name == "/DriverStation/MatchTime":
        MODE_ENTRIES[eid] = "match_time"


def read_double(record):
    """Read a double from a record, handling both float and int64-encoded doubles."""
    try:
        return record.getDouble()
    except Exception:
        pass
    try:
        return record.getFloat()
    except Exception:
        pass
    try:
        raw = record.getInteger()
        return struct.unpack('d', struct.pack('q', raw))[0]
    except Exception:
        return None


# Pass 2: collect
reader2 = DataLogReader(LOG)
cycle_ms = []
user_ms = []
log_ms = []
gc_time = []
gc_count = []
queued = []
enabled_events = []
auto_events = []
match_time_data = []

for record in reader2:
    if record.isStart() or record.isFinish() or record.isSetMetadata():
        continue
    eid = record.getEntry()
    ts = record.getTimestamp() / 1e6

    if eid in TIMING_ENTRIES:
        val = read_double(record)
        if val is not None:
            kind = TIMING_ENTRIES[eid]
            if kind == "cycle_ms":
                cycle_ms.append((ts, val))
            elif kind == "user_ms":
                user_ms.append((ts, val))
            elif kind == "log_ms":
                log_ms.append((ts, val))

    elif eid in GC_ENTRIES:
        val = read_double(record)
        if val is not None:
            kind = GC_ENTRIES[eid]
            if kind == "gc_time":
                gc_time.append((ts, val))
            elif kind == "gc_count":
                gc_count.append((ts, val))

    elif eid in QUEUED_ENTRIES:
        try:
            val = record.getInteger()
            queued.append((ts, val))
        except:
            pass

    elif eid in MODE_ENTRIES:
        kind = MODE_ENTRIES[eid]
        if kind == "enabled":
            try:
                enabled_events.append((ts, record.getBoolean()))
            except:
                pass
        elif kind == "auto":
            try:
                auto_events.append((ts, record.getBoolean()))
            except:
                pass
        elif kind == "match_time":
            val = read_double(record)
            if val is not None:
                match_time_data.append((ts, val))

print(f"Collected: {len(cycle_ms)} cycle, {len(user_ms)} user, {len(log_ms)} log, "
      f"{len(gc_time)} GC time, {len(gc_count)} GC count, {len(queued)} queued")

# Verify double decoding worked
if cycle_ms:
    print(f"Sample cycle_ms values: {[f'{v:.1f}' for _, v in cycle_ms[:5]]}")

# Find auto window
auto_start = None
auto_end = None
enable_time = None
for ts, val in sorted(enabled_events):
    if val and enable_time is None:
        enable_time = ts
        break

for ts, val in sorted(auto_events):
    if val and auto_start is None:
        auto_start = ts
    elif not val and auto_start is not None and auto_end is None:
        auto_end = ts

print(f"\nFirst enable: t={enable_time:.2f}s" if enable_time else "\nNo enable found")
print(f"Auto window: t={auto_start:.2f}s → {auto_end:.2f}s ({auto_end - auto_start:.1f}s)" if auto_start and auto_end else "Auto window not clearly bounded")

# If no auto_end, estimate from match_time (auto = 15s, match_time counts down from 15)
if auto_start and not auto_end:
    # Fallback: auto_start + 15s
    auto_end = auto_start + 15.0
    print(f"  (estimated auto_end = {auto_end:.2f}s)")

print("\n" + "=" * 80)
print("FULL CYCLE TIME ANALYSIS")
print("=" * 80)

if cycle_ms:
    all_vals = [v for _, v in cycle_ms]
    print(f"\n  Overall ({len(cycle_ms)} samples):")
    print(f"    min={min(all_vals):.1f}ms, max={max(all_vals):.1f}ms, avg={sum(all_vals)/len(all_vals):.1f}ms, median={sorted(all_vals)[len(all_vals)//2]:.1f}ms")

    for threshold in [20, 25, 30, 40, 50, 100]:
        over = [(t, v) for t, v in cycle_ms if v > threshold]
        print(f"    >{threshold}ms: {len(over)} cycles ({len(over)/len(cycle_ms)*100:.2f}%)")

    # Auto window detail
    if auto_start:
        auto_cycles = [(t, v) for t, v in cycle_ms if auto_start - 1 <= t <= (auto_end or auto_start + 20)]
        if auto_cycles:
            auto_vals = [v for _, v in auto_cycles]
            print(f"\n  AUTO PERIOD ({len(auto_cycles)} cycles, t={auto_start:.1f}–{auto_end:.1f}s):")
            print(f"    min={min(auto_vals):.1f}ms, max={max(auto_vals):.1f}ms, avg={sum(auto_vals)/len(auto_vals):.1f}ms")
            auto_over = [(t, v) for t, v in auto_cycles if v > 20]
            print(f"    >{20}ms: {len(auto_over)} cycles")
            if auto_over:
                print(f"    Overruns during auto:")
                for t, v in sorted(auto_over, key=lambda x: -x[1])[:20]:
                    print(f"      t={t:8.2f}s  cycle={v:.1f}ms")

    # Teleop first 30s
    teleop_start = auto_end if auto_end else (enable_time + 15 if enable_time else 30)
    teleop_cycles = [(t, v) for t, v in cycle_ms if teleop_start <= t <= teleop_start + 30]
    if teleop_cycles:
        teleop_vals = [v for _, v in teleop_cycles]
        print(f"\n  EARLY TELEOP ({len(teleop_cycles)} cycles, t={teleop_start:.1f}–{teleop_start+30:.1f}s):")
        print(f"    min={min(teleop_vals):.1f}ms, max={max(teleop_vals):.1f}ms, avg={sum(teleop_vals)/len(teleop_vals):.1f}ms")

# User code breakdown
print("\n" + "=" * 80)
print("USER CODE TIME")
print("=" * 80)
if user_ms and auto_start:
    auto_user = [(t, v) for t, v in user_ms if auto_start - 1 <= t <= (auto_end or auto_start + 20)]
    if auto_user:
        vals = [v for _, v in auto_user]
        print(f"  Auto period: min={min(vals):.1f}ms, max={max(vals):.1f}ms, avg={sum(vals)/len(vals):.1f}ms")
        over = [(t, v) for t, v in auto_user if v > 15]
        if over:
            print(f"  User code >15ms during auto:")
            for t, v in sorted(over, key=lambda x: -x[1])[:10]:
                print(f"    t={t:8.2f}s  user={v:.1f}ms")

# GC analysis
print("\n" + "=" * 80)
print("GARBAGE COLLECTION")
print("=" * 80)
if gc_time:
    all_gc = [v for _, v in gc_time]
    print(f"  Samples: {len(gc_time)}")
    print(f"  Total GC time range: {min(all_gc):.1f}ms → {max(all_gc):.1f}ms")

    # GC spikes (when gc_time jumps between samples)
    prev_gc = None
    gc_spikes = []
    for ts, val in gc_time:
        if prev_gc is not None:
            delta = val - prev_gc
            if delta > 1.0:  # >1ms GC in one cycle
                gc_spikes.append((ts, delta))
        prev_gc = val

    print(f"  GC spikes >1ms: {len(gc_spikes)}")
    if gc_spikes:
        # During auto
        auto_gc = [(t, d) for t, d in gc_spikes if auto_start and auto_start - 1 <= t <= (auto_end or auto_start + 20)]
        print(f"  GC spikes during auto: {len(auto_gc)}")
        for t, d in sorted(gc_spikes, key=lambda x: -x[1])[:10]:
            tag = " *** AUTO ***" if auto_start and auto_start - 1 <= t <= (auto_end or auto_start + 20) else ""
            print(f"    t={t:8.2f}s  GC={d:.1f}ms{tag}")

if gc_count:
    all_gcc = [v for _, v in gc_count]
    total_gcs = max(all_gcc) - min(all_gcc)
    print(f"\n  Total GC collections during log: {total_gcs:.0f}")
    if auto_start:
        auto_gcc = [(t, v) for t, v in gc_count if auto_start - 1 <= t <= (auto_end or auto_start + 20)]
        if len(auto_gcc) >= 2:
            auto_gcs = auto_gcc[-1][1] - auto_gcc[0][1]
            print(f"  GC collections during auto: {auto_gcs:.0f}")

# Queued cycles (backlog indicator)
print("\n" + "=" * 80)
print("LOGGER QUEUED CYCLES (backlog)")
print("=" * 80)
if queued:
    all_q = [v for _, v in queued]
    print(f"  max queued: {max(all_q)}, avg: {sum(all_q)/len(all_q):.1f}")
    high_q = [(t, v) for t, v in queued if v > 1]
    print(f"  Cycles with queue > 1: {len(high_q)}")
    if high_q and auto_start:
        auto_q = [(t, v) for t, v in high_q if auto_start - 1 <= t <= (auto_end or auto_start + 20)]
        print(f"  Queue backlog during auto: {len(auto_q)} cycles")
        for t, v in sorted(auto_q, key=lambda x: -x[1])[:10]:
            print(f"    t={t:8.2f}s  queued={v}")
