"""Probe struct record methods and raw bytes to understand how to decode ChassisSpeeds and Pose3d[]."""
from wpiutil.log import DataLogReader
import struct as pystruct

log_path = r"C:\Users\danla\Desktop\Logs2\akit_26-03-14_22-34-39_azfg_e9.wpilog"

# Find entry IDs for structs we need
targets = {
    "/RealOutputs/SwerveChassisSpeeds/Measured": "chassis",
    "/RealOutputs/Vision/Camera0/RobotPosesAccepted": "vision_accepted",
    "/RealOutputs/Vision/Camera0/RobotPosesRejected": "vision_rejected",
    "/RealOutputs/Vision/Camera0/RobotPoses": "vision_all",
    "/PowerDistribution/TotalEnergy": "pdp_energy",
    "/PowerDistribution/TotalPower": "pdp_power",
    "/PowerDistribution/TotalCurrent": "pdp_current",
}

entry_map = {}
type_map = {}
for record in DataLogReader(log_path):
    if record.isStart():
        d = record.getStartData()
        if d.name in targets:
            entry_map[d.entry] = d.name
            type_map[d.entry] = d.type

done = set()
for record in DataLogReader(log_path):
    eid = record.getEntry()
    if eid not in entry_map or eid in done:
        continue
    name = entry_map[eid]
    typ  = type_map[eid]
    done.add(eid)
    print(f"\n=== {name} (type={typ}) ===")
    # Try all read methods
    methods = ["getDouble","getFloat","getInteger","getBoolean","getString","getRaw",
               "getDoubleArray","getFloatArray","getIntegerArray","getBooleanArray","getStringArray"]
    for m in methods:
        fn = getattr(record, m, None)
        if fn:
            try:
                v = fn()
                if isinstance(v, (bytes, bytearray)):
                    print(f"  {m}() -> {len(v)} bytes | hex: {v.hex()}")
                    # Try interpreting as packed doubles (ChassisSpeeds = vx,vy,omega = 3 doubles)
                    if len(v) == 24:
                        vals = pystruct.unpack('<3d', v)
                        print(f"    -> 3 little-endian doubles: vx={vals[0]:.3f} vy={vals[1]:.3f} omega={vals[2]:.3f}")
                    elif len(v) % 8 == 0:
                        n = len(v)//8
                        vals = pystruct.unpack(f'<{n}d', v)
                        print(f"    -> {n} doubles: {[round(x,3) for x in vals]}")
                elif isinstance(v, (list, tuple)) and len(v) > 0:
                    print(f"  {m}() -> [{len(v)} items] {list(v)[:5]}")
                else:
                    print(f"  {m}() -> {v}")
            except Exception as e:
                pass
    if len(done) == len(entry_map):
        break

print("\nDone.")
