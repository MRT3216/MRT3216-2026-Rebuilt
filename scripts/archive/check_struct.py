from wpiutil.log import DataLogReader
LOG = r"C:\Users\danla\Desktop\Logs Idaho\akit_26-03-28_16-55-43_idbo_q61.wpilog"
r = DataLogReader(LOG)
entries = {}
for rec in r:
    if rec.isStart():
        d = rec.getStartData()
        entries[d.entry] = (d.name, d.type)

r2 = DataLogReader(LOG)
count = 0
for rec in r2:
    if rec.isStart() or rec.isFinish() or rec.isControl() or rec.isSetMetadata():
        continue
    eid = rec.getEntry()
    if eid not in entries:
        continue
    name, typ = entries[eid]
    if "SwerveStates/Setpoints" in name and "Optimized" not in name:
        raw = bytes(rec.getRaw())
        print(f"raw_len={len(raw)}")
        count += 1
        if count >= 3:
            break
