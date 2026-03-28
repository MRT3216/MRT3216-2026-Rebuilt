#!/usr/bin/env python3
"""Look at drive velocity during auto to see stuttering pattern, and check PP errors."""
import struct
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

# Collect drive velocity for Module 0 (FL) as a proxy for robot speed
# And cycle times for correlation
AUTO_START = 238.80
AUTO_END = 246.75

reader2 = DataLogReader(LOG)
fl_drive_vel = []
cycle_ms = []
pp_errors = []
pp_warnings = []
pp_infos = []

for rec in reader2:
    if rec.isStart() or rec.isFinish() or rec.isSetMetadata():
        continue
    eid = rec.getEntry()
    if eid not in entries:
        continue
    name = entries[eid][0]
    ts = rec.getTimestamp() / 1e6
    
    if not (AUTO_START - 2 <= ts <= AUTO_END + 2):
        continue
    
    if name == "/Drive/Module0/DriveVelocityRadPerSec":
        v = rd(rec)
        if v is not None:
            fl_drive_vel.append((ts, v))
    elif name == "/RealOutputs/LoggedRobot/FullCycleMS":
        v = rd(rec)
        if v is not None:
            cycle_ms.append((ts, v))
    elif name == "/RealOutputs/PathPlanner/errors":
        try:
            v = rec.getStringArray()
            if v:
                pp_errors.append((ts, v))
        except:
            pass
    elif name == "/RealOutputs/PathPlanner/warnings":
        try:
            v = rec.getStringArray()
            if v:
                pp_warnings.append((ts, v))
        except:
            pass
    elif name == "/RealOutputs/PathPlanner/infos":
        try:
            v = rec.getStringArray()
            if v:
                pp_infos.append((ts, v))
        except:
            pass

print(f"\n{'='*90}")
print(f"DRIVE VELOCITY vs CYCLE TIME DURING AUTO")
print(f"{'='*90}")

# Correlate: for each cycle time, find nearest drive velocity
print(f"\n  {'Time':>10s}  {'Cycle':>8s}  {'FL Vel':>10s}  Notes")
print(f"  {'-'*10}  {'-'*8}  {'-'*10}  {'-'*30}")

prev_vel = 0
for t_cyc, cyc in cycle_ms:
    if not (AUTO_START <= t_cyc <= AUTO_END):
        continue
    # Find nearest velocity
    nearest_vel = None
    min_dt = 999
    for t_v, v in fl_drive_vel:
        dt = abs(t_v - t_cyc)
        if dt < min_dt:
            min_dt = dt
            nearest_vel = v
    
    vel_str = f"{nearest_vel:10.1f}" if nearest_vel is not None else "      N/A"
    
    notes = []
    if cyc > 40:
        notes.append("*** BIG OVERRUN ***")
    elif cyc > 25:
        notes.append("* overrun *")
    
    if nearest_vel is not None and prev_vel != 0:
        vel_change = abs(nearest_vel - prev_vel) / max(abs(prev_vel), 0.1) * 100
        if vel_change > 30:
            notes.append(f"vel jump {vel_change:.0f}%")
    
    if nearest_vel is not None:
        prev_vel = nearest_vel
    
    print(f"  {t_cyc:10.2f}s  {cyc:7.1f}ms  {vel_str}  {' '.join(notes)}")

# PP messages
print(f"\n{'='*90}")
print(f"PATHPLANNER MESSAGES DURING AUTO")
print(f"{'='*90}")
if pp_errors:
    print(f"\n  ERRORS:")
    for ts, msgs in pp_errors:
        for m in msgs:
            print(f"    t={ts:.2f}s  {m}")
else:
    print(f"\n  No PP errors")

if pp_warnings:
    print(f"\n  WARNINGS:")
    for ts, msgs in pp_warnings:
        for m in msgs:
            print(f"    t={ts:.2f}s  {m}")
else:
    print(f"\n  No PP warnings")

if pp_infos:
    print(f"\n  INFOS:")
    seen = set()
    for ts, msgs in pp_infos:
        for m in msgs:
            key = m[:80]
            if key not in seen:
                print(f"    t={ts:.2f}s  {m}")
                seen.add(key)
else:
    print(f"\n  No PP infos")

# Look at velocity stability: compute jitter (variance of velocity deltas)
print(f"\n{'='*90}")
print(f"VELOCITY JITTER ANALYSIS")
print(f"{'='*90}")
auto_vel = [(t, v) for t, v in fl_drive_vel if AUTO_START <= t <= AUTO_END]
if len(auto_vel) > 2:
    deltas = []
    for i in range(1, len(auto_vel)):
        dt = auto_vel[i][0] - auto_vel[i-1][0]
        dv = auto_vel[i][1] - auto_vel[i-1][1]
        if dt > 0:
            accel = dv / dt  # rad/s^2
            deltas.append((auto_vel[i][0], dv, accel))
    
    accels = [abs(a) for _, _, a in deltas]
    print(f"  Velocity samples during auto: {len(auto_vel)}")
    print(f"  Acceleration (|dv/dt|): avg={sum(accels)/len(accels):.1f} rad/s^2, max={max(accels):.1f} rad/s^2")
    
    # Find big jerks
    big_jerks = [(t, dv, a) for t, dv, a in deltas if abs(a) > 50]
    print(f"  Acceleration > 50 rad/s^2: {len(big_jerks)}")
    for t, dv, a in big_jerks[:20]:
        print(f"    t={t:.2f}s  dv={dv:.1f} rad/s  accel={a:.1f} rad/s^2")
