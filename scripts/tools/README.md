# MRT3216 Log Analysis Tools

Reusable CLI tools for analyzing AdvantageKit `.wpilog` files from FRC matches.  
Built during Idaho/Boise 2026 event for diagnosing auto stuttering, CAN disconnects, and loop overruns.

## Setup

```bash
# From the repo root — one-time install
pip install robotpy-wpiutil
```

All tools run from the `scripts/` directory:

```bash
cd scripts
python -m tools.<tool_name> <args>
```

## Tools

### `log_inspector` — Discover What's in a Log

Explore keys, types, and sample values. Great starting point for any new log file.

```bash
# List all logged keys with type and sample count
python -m tools.log_inspector match.wpilog

# Filter to specific keys (regex)
python -m tools.log_inspector match.wpilog --filter "Drive|Flywheel"

# Just show types (compact)
python -m tools.log_inspector match.wpilog --filter "FullCycle|Enabled" --types-only

# Show match phases (AUTO/TELEOP/DISABLED with timestamps)
python -m tools.log_inspector match.wpilog --phases

# Sample decoded values from a key
python -m tools.log_inspector match.wpilog --sample "Drive/Module0/DriveVelocity" --sample-count 10
```

---

### `velocity_analyzer` — Diagnose PathPlanner Velocity Spirals

Compares chassis velocity setpoints vs measured to detect PathPlanner kP-amplified velocity spirals.

```bash
# Basic auto analysis (default)
python -m tools.velocity_analyzer match.wpilog

# Analyze teleop or all phases
python -m tools.velocity_analyzer match.wpilog --phase TELEOP

# Batch — analyze every log in a folder
python -m tools.velocity_analyzer "C:\Logs Idaho\"

# Custom robot max speed (default: 6.02 m/s from TunerConstants)
python -m tools.velocity_analyzer match.wpilog --robot-max 5.0

# Detailed timeline + per-module breakdown
python -m tools.velocity_analyzer match.wpilog --timeline --verbose
```

**What it detects:**
- Peak setpoint vs peak measured velocity
- Time spent with setpoints exceeding robot max ("stutter duration")
- Mean tracking error
- Per-module velocity breakdown (with `--verbose`)
- Verdict: `OK`, `ELEVATED`, `STUTTERING`, or `OVER MAX`

---

### `can_analyzer` — Find CAN Bus Disconnects

Detects CAN device disconnects, classifies by side (LEFT/RIGHT/CENTER), identifies cascade clusters.

```bash
# Basic analysis
python -m tools.can_analyzer match.wpilog

# Batch all logs in a folder
python -m tools.can_analyzer "C:\Logs Idaho\"

# Per-device disconnect detail
python -m tools.can_analyzer match.wpilog --detail

# Timeline of all disconnects
python -m tools.can_analyzer match.wpilog --timeline

# Adjust cascade detection window (default: 15s)
python -m tools.can_analyzer match.wpilog --min-gap 10

# CSV output for spreadsheet
python -m tools.can_analyzer match.wpilog --csv
```

**What it detects:**
- Per-device disconnect count and total downtime
- Side classification (LEFT/RIGHT/CENTER based on CAN ID module mapping)
- Cascade clusters (multiple devices disconnecting within `--min-gap` window)
- CAN bus utilization stats
- Hardware diagnosis (e.g., "RIGHT side CAN bus issue — check wiring")

---

### `timing_analyzer` — Profile Loop Overruns

Breaks down the 20ms loop budget by AdvantageKit component to find what's eating cycle time.

```bash
# Basic analysis (20ms budget default)
python -m tools.timing_analyzer match.wpilog

# Custom budget
python -m tools.timing_analyzer match.wpilog --budget 25

# Batch analysis
python -m tools.timing_analyzer "C:\Logs Idaho\"
```

**What it reports:**
- Per-phase (AUTO/TELEOP) timing statistics
- Overrun percentage (cycles exceeding budget)
- Breakdown: UserCode vs LogPeriodic vs GC vs Other
- Logger sub-breakdown (CondSave, DS, AlertLog, CondCapture, etc.)
- Mean, median, P95, P99, max for each component

---

## Shared Library: `wpilog_utils.py`

All tools build on `LogReader` which handles:

- **AdvantageKit double decoding** — AK stores doubles as int64 bit patterns (`struct.unpack('d', struct.pack('q', raw))`)
- **Phase detection** — AUTO / TELEOP / DISABLED with precise timestamps from `/DriverStation/Enabled` and `/DriverStation/Autonomous`
- **Struct decoding** — `SwerveModuleState` (16 bytes, `<dd`), `ChassisSpeeds` (24 bytes, `<ddd`)
- **Device classification** — CAN ID → module position, side, device name
- **Phoenix signal caching** — Handles StatusSignal patterns

### Using in your own script

```python
import sys
sys.path.insert(0, "path/to/scripts")

from tools.wpilog_utils import LogReader

reader = LogReader("match.wpilog")

# Get all entries for a key
for ts, value in reader.get_doubles("RealOutputs/SwerveSetpoint/vxMetersPerSecond"):
    print(f"{ts:.3f}s  vx={value:.2f}")

# Get match phases
for phase in reader.get_phases():
    print(f"{phase.name}: {phase.start:.1f}s - {phase.end:.1f}s")

# Decode struct arrays
for ts, states in reader.get_struct_array("Drive/Module0/...", struct_size=16, fmt="<dd"):
    speed, angle = states[0]  # first struct in array
```

## Archive

One-off analysis scripts from Idaho/Boise 2026 are preserved in `scripts/archive/`. These were the original investigation scripts that led to the creation of these reusable tools.
