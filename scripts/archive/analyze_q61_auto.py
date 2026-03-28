#!/usr/bin/env python3
"""Focused Q61 auto timing analysis with correct phase detection."""
import struct
from wpiutil.log import DataLogReader

LOG = r"C:\Users\danla\Desktop\Logs Idaho\akit_26-03-28_16-55-43_idbo_q61.wpilog"

def rd(rec):
    try:
        return rec.getDouble()
    except Exception:
        pass
    try:
        raw = rec.getInteger()
        return struct.unpack('d', struct.pack('q', raw))[0]
    except Exception:
        return None

print("Reading Q61...")
reader = DataLogReader(LOG)
entries = {}
for rec in reader:
    if rec.isStart():
        d = rec.getStartData()
        entries[d.entry] = (d.name, d.type)

# Collect everything in one pass
reader2 = DataLogReader(LOG)
cycle_ms = []
user_ms = []
gc_time_ms = []
gc_count_vals = []

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
        if v is not None:
            cycle_ms.append((ts, v))
    elif name == "/RealOutputs/LoggedRobot/UserCodeMS":
        v = rd(rec)
        if v is not None:
            user_ms.append((ts, v))
    elif name == "/RealOutputs/LoggedRobot/GCTimeMS":
        v = rd(rec)
        if v is not None:
            gc_time_ms.append((ts, v))
    elif name == "/RealOutputs/LoggedRobot/GCCounts":
        v = rd(rec)
        if v is not None:
            gc_count_vals.append((ts, v))

# Match phases from DS flags:
# Auto=True at t=29.65, Enabled=True at t=238.80
# Enabled=False at t=246.75 (brief disable?), Enabled=True at t=263.51
# Auto=False at t=263.51, Enabled=True at t=263.51 (teleop start)
# Enabled=False at t=403.60 (match end)

# Actual auto = t=238.80 to t=246.75 (first enabled window while auto=True) = 7.95s
# Then disabled t=246.75 to t=263.51
# Teleop = t=263.51 to t=403.60

# But the big overruns are at t=29-37. That's when Auto=True first set but Enabled=False.
# This is the DISABLED period when PathPlanner warmup should be running!

print("\n" + "="*80)
print("MATCH PHASE TIMELINE")
print("="*80)
print("  t=  1.69s  Boot (Enabled=False, Auto=False)")
print("  t= 29.65s  Auto=True (but still DISABLED — warmup/class-loading period)")
print("  t=238.80s  Enabled=True (AUTO STARTS — robot moves)")
print("  t=246.75s  Enabled=False (brief disable — E-stop or brownout?)")
print("  t=263.51s  Auto=False, Enabled=True (TELEOP STARTS)")
print("  t=403.60s  Enabled=False (MATCH END)")

auto_start = 238.80
auto_end = 246.75  # first disable
teleop_start = 263.51
match_end = 403.60
warmup_start = 29.65
warmup_end = 238.80

# ── Warmup period (t=29-238, disabled but auto mode) ────────────────
print("\n" + "="*80)
print(f"WARMUP/DISABLED PERIOD (t={warmup_start:.0f}–{auto_start:.0f}s)")
print("="*80)
warmup_cycles = [(t, v) for t, v in cycle_ms if warmup_start <= t <= auto_start]
if warmup_cycles:
    vals = [v for _, v in warmup_cycles]
    print(f"  {len(warmup_cycles)} cycles, min={min(vals):.1f}ms, max={max(vals):.1f}ms, avg={sum(vals)/len(vals):.1f}ms")
    over = [(t, v) for t, v in warmup_cycles if v > 20]
    print(f"  >20ms: {len(over)} cycles")
    print(f"\n  Worst overruns during warmup:")
    for t, v in sorted(over, key=lambda x: -x[1])[:15]:
        print(f"    t={t:8.2f}s  {v:.1f}ms")

# ── Actual Auto (t=238.80–246.75) ───────────────────────────────────
print("\n" + "="*80)
print(f"ACTUAL AUTO PERIOD (t={auto_start:.1f}–{auto_end:.1f}s, {auto_end-auto_start:.1f}s)")
print("="*80)
auto_cycles = [(t, v) for t, v in cycle_ms if auto_start <= t <= auto_end]
if auto_cycles:
    vals = [v for _, v in auto_cycles]
    print(f"  {len(auto_cycles)} cycles, min={min(vals):.1f}ms, max={max(vals):.1f}ms, avg={sum(vals)/len(vals):.1f}ms")
    over = [(t, v) for t, v in auto_cycles if v > 20]
    print(f"  >20ms: {len(over)} cycles ({len(over)/len(auto_cycles)*100:.1f}%)")
    
    print(f"\n  ALL cycle times during auto:")
    for t, v in auto_cycles:
        marker = " ***" if v > 20 else (" *" if v > 17 else "")
        print(f"    t={t:8.2f}s  cycle={v:.1f}ms{marker}")
else:
    print("  No cycle data in this window!")

# User code during auto
auto_user = [(t, v) for t, v in user_ms if auto_start <= t <= auto_end]
if auto_user:
    vals = [v for _, v in auto_user]
    print(f"\n  User code: min={min(vals):.1f}ms, max={max(vals):.1f}ms, avg={sum(vals)/len(vals):.1f}ms")
    over = [(t, v) for t, v in auto_user if v > 15]
    if over:
        print(f"  User code >15ms:")
        for t, v in sorted(over, key=lambda x: -x[1]):
            print(f"    t={t:8.2f}s  user={v:.1f}ms")

# ── Brief disable (t=246.75–263.51) ─────────────────────────────────
print("\n" + "="*80)
print(f"BRIEF DISABLE (t={auto_end:.1f}–{teleop_start:.1f}s, {teleop_start-auto_end:.1f}s)")
print("="*80)
disable_cycles = [(t, v) for t, v in cycle_ms if auto_end <= t <= teleop_start]
if disable_cycles:
    vals = [v for _, v in disable_cycles]
    print(f"  {len(disable_cycles)} cycles, min={min(vals):.1f}ms, max={max(vals):.1f}ms, avg={sum(vals)/len(vals):.1f}ms")

# ── Teleop ──────────────────────────────────────────────────────────
print("\n" + "="*80)
print(f"TELEOP (t={teleop_start:.1f}–{match_end:.1f}s)")
print("="*80)
teleop_cycles = [(t, v) for t, v in cycle_ms if teleop_start <= t <= match_end]
if teleop_cycles:
    vals = [v for _, v in teleop_cycles]
    print(f"  {len(teleop_cycles)} cycles, min={min(vals):.1f}ms, max={max(vals):.1f}ms, avg={sum(vals)/len(vals):.1f}ms")
    over20 = sum(1 for v in vals if v > 20)
    over40 = sum(1 for v in vals if v > 40)
    print(f"  >20ms: {over20} ({over20/len(vals)*100:.1f}%)")
    print(f"  >40ms: {over40} ({over40/len(vals)*100:.1f}%)")

# ── GC during auto ──────────────────────────────────────────────────
print("\n" + "="*80)
print("GARBAGE COLLECTION DURING AUTO")
print("="*80)
auto_gc = [(t, v) for t, v in gc_time_ms if auto_start <= t <= auto_end]
if auto_gc:
    prev = None
    for t, v in auto_gc:
        if prev is not None:
            delta = v - prev
            if delta > 0:
                print(f"  t={t:8.2f}s  GC delta={delta:.0f}ms")
        prev = v
else:
    print("  No GC data during auto window (GC logged less frequently)")

# Check GC around auto time
print("\n  GC around auto time (t=230-270):")
near_gc = [(t, v) for t, v in gc_time_ms if 230 <= t <= 270]
prev = None
for t, v in near_gc:
    if prev is not None:
        delta = v - prev[1]
        if delta > 0:
            print(f"    t={t:8.2f}s  GC +{delta:.0f}ms (cumulative={v:.0f}ms)")
    prev = (t, v)

# Check first GC spike at startup
print("\n  GC around startup (t=28-40):")
start_gc = [(t, v) for t, v in gc_time_ms if 25 <= t <= 42]
prev = None
for t, v in start_gc:
    if prev is not None:
        delta = v - prev[1]
        if delta > 0:
            print(f"    t={t:8.2f}s  GC +{delta:.0f}ms (cumulative={v:.0f}ms)")
    prev = (t, v)
