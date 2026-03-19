from wpiutil.log import DataLogReader
from collections import defaultdict

log_path = r"C:\Users\danla\Desktop\Logs2\akit_26-03-14_22-34-39_azfg_e9.wpilog"
reader = DataLogReader(log_path)

SIGNALS = {
    "/SystemStats/BatteryVoltage",
    "/SystemStats/BatteryCurrent",
    "/PowerDistribution/TotalCurrent",
    "/Intake/Pivot/Current",
    "/Intake/Pivot/Volts",
    "/Intake/Pivot/Angle",
    "/Intake/Pivot/Setpoint",
    "/RealOutputs/CommandsAll/IntakePivotSubsystem SetDutyCycle",
    "/RealOutputs/CommandsAll/Intake.Deploy",
    "/Shooter/Turret/Current",
    "/Shooter/Turret/Angle",
    "/Shooter/Turret/Setpoint",
    "/Hood/Current",
    "/Hood/Angle",
    "/Hood/Setpoint",
    "/Shooter/Flywheel/Current",
    "/DriverStation/Enabled",
}

# Build entry_id -> signal_name map
entry_map = {}
type_map = {}
reader2 = DataLogReader(log_path)
for record in reader2:
    if record.isStart():
        d = record.getStartData()
        if d.name in SIGNALS:
            entry_map[d.entry] = d.name
            type_map[d.entry] = d.type

# Collect data
data = defaultdict(list)
reader3 = DataLogReader(log_path)
for record in reader3:
    entry_id = record.getEntry()
    if entry_id in entry_map:
        name = entry_map[entry_id]
        t = record.getTimestamp() / 1e6  # microseconds -> seconds
        typ = type_map[entry_id]
        try:
            if typ == "double" or typ == "float":
                val = record.getDouble()
            elif typ == "boolean":
                val = record.getBoolean()
            elif typ == "int64":
                val = record.getInteger()
            else:
                continue
            data[name].append((t, val))
        except Exception:
            pass

# --- Summary stats ---
print("=" * 60)
print(f"LOG: {log_path.split(chr(92))[-1]}")
print("=" * 60)

def stats(name):
    pts = data.get(name, [])
    if not pts:
        return "  NO DATA"
    vals = [v for _, v in pts]
    return f"  n={len(vals):5d}  min={min(vals):8.3f}  max={max(vals):8.3f}  avg={sum(vals)/len(vals):8.3f}"

for sig in sorted(SIGNALS):
    print(f"\n{sig}")
    print(stats(sig))

# --- Battery voltage sag: find worst 5-second window ---
print("\n" + "=" * 60)
print("BATTERY VOLTAGE - lowest sustained readings:")
bv = data.get("/SystemStats/BatteryVoltage", [])
if bv:
    bv.sort()
    low = sorted(bv, key=lambda x: x[1])[:10]
    for t, v in low:
        print(f"  t={t:8.2f}s  voltage={v:.3f}V")

# --- Intake pivot: show current while at setpoint=0 (default command) ---
print("\n" + "=" * 60)
print("INTAKE PIVOT - current draw over time (sample every ~2s):")
pivot_current = data.get("/Intake/Pivot/Current", [])
if pivot_current:
    pivot_current.sort(key=lambda x: x[0])
    step = max(1, len(pivot_current) // 30)
    for t, v in pivot_current[::step]:
        print(f"  t={t:8.2f}s  current={v:.2f}A")

# --- Turret: position vs setpoint ---
print("\n" + "=" * 60)
print("TURRET - angle vs setpoint (sample every ~2s):")
t_angle = dict(data.get("/Shooter/Turret/Angle", []))
t_setpt = dict(data.get("/Shooter/Turret/Setpoint", []))
times = sorted(set(list(t_angle.keys()) + list(t_setpt.keys())))
step = max(1, len(times) // 20)
for t in times[::step]:
    ang = t_angle.get(t, float("nan"))
    sp = t_setpt.get(t, float("nan"))
    print(f"  t={t:8.2f}s  angle={ang:8.2f}  setpoint={sp:8.2f}  error={ang-sp:+.2f}")
