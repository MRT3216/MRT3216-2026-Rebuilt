#!/usr/bin/env python3
"""Analyze Q61 for auto-period stuttering: loop overruns, timing, and CAN events."""
import sys
from collections import defaultdict
from wpiutil.log import DataLogReader

LOG = r"C:\Users\danla\Desktop\Logs Idaho\akit_26-03-28_16-55-43_idbo_q61.wpilog"
print(f"Reading Q61 log...")

reader = DataLogReader(LOG)
entries = {}
for record in reader:
    if record.isStart():
        d = record.getStartData()
        entries[d.entry] = (d.name, d.type)

# Find relevant entry IDs
target_keys = {}
for eid, (name, typ) in entries.items():
    nl = name.lower()
    if "loopoverruncount" in nl or "loopoverrun" in nl:
        target_keys[eid] = ("overrun_count", name, typ)
    elif name == "/RealOutputs/LoggedRobot/FullCycleMS":
        target_keys[eid] = ("cycle_ms", name, typ)
    elif name == "/RealOutputs/LoggedRobot/LogPeriodicMS":
        target_keys[eid] = ("log_ms", name, typ)
    elif name == "/RealOutputs/LoggedRobot/UserCodeMS":
        target_keys[eid] = ("user_ms", name, typ)
    elif "realoutputs" in nl and "cyclems" in nl.replace("/", "").replace("_", ""):
        target_keys[eid] = ("other_cycle", name, typ)
    elif "connected" in nl and ("drive" in nl or "module" in nl):
        target_keys[eid] = ("can_conn", name, typ)
    elif name == "/DSLog/RobotEnabled":
        target_keys[eid] = ("enabled", name, typ)
    elif "driverstation" in nl and "enabled" in nl:
        target_keys[eid] = ("ds_enabled", name, typ)
    elif "autonomous" in nl and ("active" in nl or "running" in nl):
        target_keys[eid] = ("auto_active", name, typ)
    elif name == "/RealOutputs/LoggedRobot/OverrunCycleCount":
        target_keys[eid] = ("overrun_cycle_count", name, typ)

# Print all timing-related entries for discovery
print("\n--- Timing/loop related entries ---")
for eid, (name, typ) in sorted(entries.items(), key=lambda x: x[1][0]):
    nl = name.lower()
    if any(k in nl for k in ["cycle", "overrun", "loop", "periodic", "usercode", "timing", "logged"]):
        print(f"  [{typ:10s}] {name}")

print("\n--- DS/mode related entries ---")
for eid, (name, typ) in sorted(entries.items(), key=lambda x: x[1][0]):
    nl = name.lower()
    if any(k in nl for k in ["dslog", "driverstation", "enabled", "autonomous", "teleop", "mode"]):
        print(f"  [{typ:10s}] {name}")

# Pass 2: collect data
reader2 = DataLogReader(LOG)
cycle_times = []  # (ts, ms)
user_times = []
log_times = []
can_disconnects = []
overrun_counts = []
last_can_state = {}

for record in reader2:
    if record.isStart() or record.isFinish() or record.isSetMetadata():
        continue
    eid = record.getEntry()
    if eid not in target_keys and eid not in entries:
        continue
    ts = record.getTimestamp() / 1e6

    if eid in target_keys:
        kind, name, typ = target_keys[eid]
        if kind == "cycle_ms":
            try:
                val = record.getInteger()
                cycle_times.append((ts, val))
            except:
                try:
                    val = record.getDouble()
                    cycle_times.append((ts, val))
                except: pass
        elif kind == "user_ms":
            try:
                val = record.getInteger()
                user_times.append((ts, val))
            except:
                try:
                    val = record.getDouble()
                    user_times.append((ts, val))
                except: pass
        elif kind == "log_ms":
            try:
                val = record.getInteger()
                log_times.append((ts, val))
            except:
                try:
                    val = record.getDouble()
                    log_times.append((ts, val))
                except: pass
        elif kind in ("overrun_count", "overrun_cycle_count"):
            try:
                val = record.getInteger()
                overrun_counts.append((ts, val, name))
            except: pass
        elif kind == "can_conn":
            try:
                val = record.getBoolean()
                prev = last_can_state.get(eid)
                if prev is not None and prev != val and not val:
                    can_disconnects.append((ts, name))
                last_can_state[eid] = val
            except: pass

print(f"\n--- Data collected ---")
print(f"  Cycle time samples: {len(cycle_times)}")
print(f"  User code samples:  {len(user_times)}")
print(f"  Log period samples: {len(log_times)}")
print(f"  Overrun counters:   {len(overrun_counts)}")
print(f"  CAN drive disconnects: {len(can_disconnects)}")

# Auto period is roughly t=0 to t=15-18s after enable
# Find enable time by looking for first cycle time after boot
if cycle_times:
    first_cycle_ts = cycle_times[0][0]
    print(f"\n  First cycle data at t={first_cycle_ts:.2f}s")

# Analyze cycle times during auto window (let's look at first 30s of data and around typical auto)
print("\n" + "="*80)
print("CYCLE TIME ANALYSIS")
print("="*80)

if cycle_times:
    all_vals = [v for _, v in cycle_times]
    print(f"\n  Overall: min={min(all_vals)}ms, max={max(all_vals)}ms, avg={sum(all_vals)/len(all_vals):.1f}ms")
    over_20 = [(t, v) for t, v in cycle_times if v > 20]
    over_40 = [(t, v) for t, v in cycle_times if v > 40]
    over_100 = [(t, v) for t, v in cycle_times if v > 100]
    print(f"  Cycles >20ms: {len(over_20)} ({len(over_20)/len(cycle_times)*100:.1f}%)")
    print(f"  Cycles >40ms: {len(over_40)} ({len(over_40)/len(cycle_times)*100:.1f}%)")
    print(f"  Cycles >100ms: {len(over_100)} ({len(over_100)/len(cycle_times)*100:.1f}%)")

    # Show worst overruns
    print(f"\n  Top 20 worst cycle times:")
    sorted_cycles = sorted(cycle_times, key=lambda x: -x[1])[:20]
    for ts, ms in sorted_cycles:
        print(f"    t={ts:8.2f}s  {ms:6d}ms")

    # Auto window analysis (roughly first 20s after first significant data)
    # Look for a cluster of activity - auto typically starts 1-3s into log
    print(f"\n  Cycle times in first 30s (likely auto period):")
    early = [(t, v) for t, v in cycle_times if t < 30]
    if early:
        vals = [v for _, v in early]
        print(f"    samples={len(early)}, min={min(vals)}ms, max={max(vals)}ms, avg={sum(vals)/len(vals):.1f}ms")
        early_over_20 = [(t, v) for t, v in early if v > 20]
        print(f"    cycles >20ms: {len(early_over_20)}")
        for t, v in sorted(early_over_20, key=lambda x: -x[1])[:10]:
            print(f"      t={t:8.2f}s  {v}ms")

    # 15-50s window
    print(f"\n  Cycle times t=15-50s (auto to early teleop):")
    mid = [(t, v) for t, v in cycle_times if 15 <= t <= 50]
    if mid:
        vals = [v for _, v in mid]
        print(f"    samples={len(mid)}, min={min(vals)}ms, max={max(vals)}ms, avg={sum(vals)/len(vals):.1f}ms")
        mid_over_20 = [(t, v) for t, v in mid if v > 20]
        print(f"    cycles >20ms: {len(mid_over_20)}")
        for t, v in sorted(mid_over_20, key=lambda x: -x[1])[:10]:
            print(f"      t={t:8.2f}s  {v}ms")

if user_times:
    print(f"\n  User code time: min={min(v for _,v in user_times)}ms, max={max(v for _,v in user_times)}ms, avg={sum(v for _,v in user_times)/len(user_times):.1f}ms")
    user_over_10 = [(t, v) for t, v in user_times if v > 10]
    print(f"  User code >10ms: {len(user_over_10)}")
    for t, v in sorted(user_over_10, key=lambda x: -x[1])[:10]:
        print(f"    t={t:8.2f}s  {v}ms")

if log_times:
    print(f"\n  Log periodic time: min={min(v for _,v in log_times)}ms, max={max(v for _,v in log_times)}ms, avg={sum(v for _,v in log_times)/len(log_times):.1f}ms")
    log_over_10 = [(t, v) for t, v in log_times if v > 10]
    print(f"  Log periodic >10ms: {len(log_over_10)}")
    for t, v in sorted(log_over_10, key=lambda x: -x[1])[:5]:
        print(f"    t={t:8.2f}s  {v}ms")

# Overrun counter
if overrun_counts:
    print(f"\n" + "="*80)
    print("OVERRUN COUNTERS")
    print("="*80)
    by_name = defaultdict(list)
    for ts, val, name in overrun_counts:
        by_name[name].append((ts, val))
    for name, samples in by_name.items():
        vals = [v for _, v in samples]
        print(f"  {name}: final={vals[-1]}, max={max(vals)}")
        # Show when it increases
        prev_val = 0
        for ts, val in samples:
            if val > prev_val:
                print(f"    +{val - prev_val} at t={ts:.2f}s (total={val})")
                prev_val = val

# CAN disconnects during auto
print(f"\n" + "="*80)
print("CAN DRIVE DISCONNECTS")
print("="*80)
if can_disconnects:
    for ts, name in sorted(can_disconnects):
        tag = " *** AUTO ***" if ts < 30 else ""
        print(f"  t={ts:8.2f}s  {name}{tag}")
else:
    print("  None!")
