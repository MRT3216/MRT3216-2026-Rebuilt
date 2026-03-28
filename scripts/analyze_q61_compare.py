#!/usr/bin/env python3
"""Compare auto vs teleop timing in Q61 — what makes auto different?"""
import struct
from collections import defaultdict
from wpiutil.log import DataLogReader

LOG = r"C:\Users\danla\Desktop\Logs Idaho\akit_26-03-28_16-55-43_idbo_q61.wpilog"

def rd(rec):
    try: return rec.getDouble()
    except:
        try: return struct.unpack('d', struct.pack('q', rec.getInteger()))[0]
        except: return None

print("Reading Q61...")
reader = DataLogReader(LOG)
entries = {}
for rec in reader:
    if rec.isStart():
        d = rec.getStartData()
        entries[d.entry] = (d.name, d.type)

reader2 = DataLogReader(LOG)
cycle_ms = []
user_ms = []
log_ms = []
gc_time = []

for rec in reader2:
    if rec.isStart() or rec.isFinish() or rec.isSetMetadata():
        continue
    eid = rec.getEntry()
    if eid not in entries:
        continue
    name = entries[eid][0]
    ts = rec.getTimestamp() / 1e6
    if name == "/RealOutputs/LoggedRobot/FullCycleMS":
        v = rd(rec)
        if v is not None: cycle_ms.append((ts, v))
    elif name == "/RealOutputs/LoggedRobot/UserCodeMS":
        v = rd(rec)
        if v is not None: user_ms.append((ts, v))
    elif name == "/RealOutputs/LoggedRobot/LogPeriodicMS":
        v = rd(rec)
        if v is not None: log_ms.append((ts, v))
    elif name == "/RealOutputs/LoggedRobot/GCTimeMS":
        v = rd(rec)
        if v is not None: gc_time.append((ts, v))

# Phases from DS flags:
# Auto enabled: 238.80 -> 246.75
# Teleop: 263.51 -> 403.60
AUTO_START = 238.80
AUTO_END = 246.75
TELEOP_START = 263.51
TELEOP_END = 403.60

def analyze_window(data, t0, t1, label):
    window = [(t, v) for t, v in data if t0 <= t <= t1]
    if not window:
        print(f"  {label}: no data")
        return
    vals = [v for _, v in window]
    vals.sort()
    n = len(vals)
    p50 = vals[n // 2]
    p90 = vals[int(n * 0.90)]
    p95 = vals[int(n * 0.95)]
    p99 = vals[int(n * 0.99)]
    over20 = sum(1 for v in vals if v > 20)
    over25 = sum(1 for v in vals if v > 25)
    over30 = sum(1 for v in vals if v > 30)
    over40 = sum(1 for v in vals if v > 40)

    print(f"  {label} ({n} cycles, {t1-t0:.1f}s):")
    print(f"    avg={sum(vals)/n:.1f}ms  p50={p50:.1f}ms  p90={p90:.1f}ms  p95={p95:.1f}ms  p99={p99:.1f}ms  max={max(vals):.1f}ms")
    print(f"    >20ms: {over20} ({over20/n*100:.1f}%)  >25ms: {over25} ({over25/n*100:.1f}%)  >30ms: {over30} ({over30/n*100:.1f}%)  >40ms: {over40} ({over40/n*100:.1f}%)")

print("\n" + "="*90)
print("FULL CYCLE TIME (FullCycleMS)")
print("="*90)
analyze_window(cycle_ms, AUTO_START, AUTO_END, "AUTO")
analyze_window(cycle_ms, TELEOP_START, TELEOP_END, "TELEOP")
# Split teleop into early/mid/late
teleop_dur = TELEOP_END - TELEOP_START
analyze_window(cycle_ms, TELEOP_START, TELEOP_START + teleop_dur/3, "TELEOP early")
analyze_window(cycle_ms, TELEOP_START + teleop_dur/3, TELEOP_START + 2*teleop_dur/3, "TELEOP mid")
analyze_window(cycle_ms, TELEOP_START + 2*teleop_dur/3, TELEOP_END, "TELEOP late")

print("\n" + "="*90)
print("USER CODE TIME (UserCodeMS)")
print("="*90)
analyze_window(user_ms, AUTO_START, AUTO_END, "AUTO")
analyze_window(user_ms, TELEOP_START, TELEOP_END, "TELEOP")

print("\n" + "="*90)
print("LOG PERIODIC TIME (LogPeriodicMS)")
print("="*90)
analyze_window(log_ms, AUTO_START, AUTO_END, "AUTO")
analyze_window(log_ms, TELEOP_START, TELEOP_END, "TELEOP")

# GC during auto vs teleop
print("\n" + "="*90)
print("GARBAGE COLLECTION COMPARISON")
print("="*90)

def gc_spikes_in_window(data, t0, t1):
    window = [(t, v) for t, v in data if t0 - 5 <= t <= t1 + 5]  # slight margin
    spikes = []
    prev = None
    for t, v in window:
        if prev is not None and t0 <= t <= t1:
            delta = v - prev
            if delta > 0:
                spikes.append((t, delta))
        prev = v
    return spikes

auto_gc = gc_spikes_in_window(gc_time, AUTO_START, AUTO_END)
teleop_gc = gc_spikes_in_window(gc_time, TELEOP_START, TELEOP_END)

print(f"  AUTO  ({AUTO_END-AUTO_START:.1f}s): {len(auto_gc)} GC events, total={sum(d for _,d in auto_gc):.0f}ms")
for t, d in auto_gc:
    print(f"    t={t:.2f}s  +{d:.0f}ms")

print(f"  TELEOP ({TELEOP_END-TELEOP_START:.1f}s): {len(teleop_gc)} GC events, total={sum(d for _,d in teleop_gc):.0f}ms")
if teleop_gc:
    avg_interval = (TELEOP_END - TELEOP_START) / len(teleop_gc)
    print(f"    avg interval: {avg_interval:.1f}s, avg GC: {sum(d for _,d in teleop_gc)/len(teleop_gc):.0f}ms")
    big = [(t, d) for t, d in teleop_gc if d > 10]
    print(f"    GC > 10ms: {len(big)}")
    for t, d in big[:10]:
        print(f"      t={t:.2f}s  +{d:.0f}ms")

# Distribution of cycle times — histogram
print("\n" + "="*90)
print("CYCLE TIME DISTRIBUTION (histogram)")
print("="*90)
for label, t0, t1 in [("AUTO", AUTO_START, AUTO_END), ("TELEOP", TELEOP_START, TELEOP_END)]:
    window = [v for t, v in cycle_ms if t0 <= t <= t1]
    if not window:
        continue
    n = len(window)
    buckets = [(0,15), (15,17), (17,19), (19,20), (20,22), (22,25), (25,30), (30,40), (40,60), (60,100)]
    print(f"\n  {label}:")
    for lo, hi in buckets:
        count = sum(1 for v in window if lo <= v < hi)
        pct = count / n * 100
        bar = "#" * int(pct)
        print(f"    {lo:3d}-{hi:3d}ms: {count:4d} ({pct:5.1f}%) {bar}")
    over = sum(1 for v in window if v >= 100)
    if over:
        print(f"    100+ ms: {over:4d} ({over/n*100:5.1f}%)")

# Consecutive overruns during auto (stutter = multiple bad cycles in a row)
print("\n" + "="*90)
print("CONSECUTIVE OVERRUN STREAKS DURING AUTO (cycles >20ms in a row)")
print("="*90)
auto_cyc = [(t, v) for t, v in cycle_ms if AUTO_START <= t <= AUTO_END]
streaks = []
current_streak = []
for t, v in auto_cyc:
    if v > 20:
        current_streak.append((t, v))
    else:
        if len(current_streak) >= 3:
            streaks.append(current_streak)
        current_streak = []
if len(current_streak) >= 3:
    streaks.append(current_streak)

print(f"  Streaks of 3+ consecutive overruns: {len(streaks)}")
for i, streak in enumerate(streaks):
    total_ms = sum(v for _, v in streak)
    print(f"\n  Streak {i+1}: {len(streak)} cycles, t={streak[0][0]:.2f}–{streak[-1][0]:.2f}s, total={total_ms:.0f}ms (expected={len(streak)*20}ms, excess={total_ms-len(streak)*20:.0f}ms)")
    for t, v in streak:
        print(f"    t={t:.2f}s  {v:.1f}ms")
