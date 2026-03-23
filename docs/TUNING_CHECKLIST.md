# Tuning and Calibration Checklist

> **How to use:** Copy this into a GitHub Issue (or paste into GitHub Projects) to track progress.
> Check off items as you complete them. Items are in dependency order - complete earlier items before later ones.
>
> **Reference:** See docs/TuningGuide.md for detailed procedures, gain tables, and AdvantageScope tips.

---

## Priority 0 - Pre-Tuning Setup

### Safety and Environment

- [ ] Robot on blocks (wheels off ground) for Steps 0-2, 4
- [ ] Someone on the E-stop at all times
- [ ] Constants.tuningMode = true (enables HIGH telemetry + live tuning)
- [ ] AdvantageScope connected and receiving data
- [ ] Phoenix Tuner X connected - all TalonFX + CANcoders green
- [ ] REV Hardware Client connected - all SparkMax/SparkFlex visible
- [ ] Elastic Dashboard loaded with elastic-layout.json

### Physical Measurements

- [ ] Weigh robot with bumpers + battery then update Constants.DriveConstants.kRobotMassKg, Drive.java, and settings.json
- [ ] Verify module encoder offsets - all wheels point straight forward
- [ ] Verify motor inversions - forward = positive velocity, CCW steer = positive position
- [ ] Verify CAN bus health - 0 percent error rate, CANFD bus confirmed
- [ ] Verify firmware - all Phoenix 6 devices on v26, REV devices up to date
- [ ] Inspect Pigeon 2 mounting then configure MountPose if needed (see Pigeon IMU Calibration in TuningGuide)

---

## Priority 1 - Drivetrain (Steps 0-8)

> **Time budget:** 2-4 hours. Everything else depends on drive/odometry being accurate.

### Step 0: Pre-flight Checks

- [ ] All 8 TalonFX (4 drive + 4 steer) + 4 CANcoders visible in Tuner X
- [ ] Module offsets correct (wheels straight when zeroed)
- [ ] Motor inversions correct
- [ ] CAN bus healthy (0 percent error rate)
- [ ] Robot on blocks

### Step 1: Steer PID

> Current: kP=100, kI=0, kD=0.5, kS=0.1, kV=2.48, kA=0 (MotionMagicExpo)

- [ ] Verify kS - module barely moves at steer threshold voltage
- [ ] Verify kV - cruise slope matches between Position and Reference
- [ ] Verify kP - 0 deg to 90 deg settles in under 50ms, no oscillation
- [ ] Verify kD - no overshoot, no buzzing
- [ ] Plot SwerveStates/Measured vs SwerveStates/SetpointsOptimized - angles track cleanly

### Step 2: Drive Feedforward (kS, kV)

> Current: kP=0.1, kI=0, kD=0, kS=0.26, kV=0.585

- [ ] Uncomment setupSysid() in RobotContainer constructor
- [ ] Deploy, select Drive Simple FF Characterization from auto chooser
- [ ] Run characterization (robot drives forward with slow ramp, 5-10 sec)
- [ ] Record kS and kV from Driver Station console
- [ ] Repeat 2-3 times and average results
- [ ] Update driveGains in TunerConstants.java
- [ ] (Optional) Run full SysId (4 routines) for kA
- [ ] Re-comment setupSysid() when done

### Step 3: Wheel Radius Characterization

> Current: kWheelRadius = 1.8 inches (nominal - needs carpet characterization)

- [ ] Place robot on carpet (not hard floor)
- [ ] Select Drive Wheel Radius Characterization from auto chooser
- [ ] Run 2-3 full rotations, disable
- [ ] Record measured radius from DS console
- [ ] Update kWheelRadius in TunerConstants.java
- [ ] Update driveWheelRadius in settings.json (PathPlanner)

### Step 4: Drive PID (kP)

> Current: kP=0.1

- [ ] Teleop: sudden joystick push (0 to approx 2 m/s)
- [ ] Plot measured vs setpoint velocities in AdvantageScope
- [ ] Adjust kP - rise time under 100ms with no oscillation
- [ ] Verify: oscillation means reduce kP; steady-state error means FF problem not PID

### Step 5: Max Speed Measurement

> Current: kSpeedAt12Volts = 6.02 m/s (theoretical - expected actual approx 5.5 m/s)

- [ ] Open space (30+ ft), fully charged battery
- [ ] Drive full speed forward, hold 3+ seconds
- [ ] Read peak sustained velocity in AdvantageScope (plateau, not spike)
- [ ] Update kSpeedAt12Volts in TunerConstants.java
- [ ] Update maxDriveSpeed in settings.json (PathPlanner)

### Step 6: Slip Current Measurement

> Current: kSlipCurrent = 120A (reference teams use 80-95A)

- [ ] Push robot against solid wall, all wheels on carpet
- [ ] Plot drive stator current and velocity in AdvantageScope
- [ ] Slowly ramp joystick 0 to 100 percent into wall
- [ ] Note current at moment velocity jumps (wheels lose traction)
- [ ] Repeat 2-3 times, average
- [ ] Update kSlipCurrent in TunerConstants.java
- [ ] Verify: if measured value is under stator limit (80A), traction control will engage

### Step 7: Odometry Frequency

- [ ] Verify CAN FD odometry runs at 250 Hz (no action unless loop overruns observed)

### Step 8: PathPlanner Configuration

- [ ] Verify settings.json matches code for: robotMass, robotMOI, wheelCOF, driveWheelRadius, maxDriveSpeed, driveCurrentLimit
- [ ] Test Translation kP (5.0) with straight path - reduce to 3.0 if oscillating
- [ ] Test Rotation kP (5.0) with turning path - reduce to 3.0 if oscillating
- [ ] Verify MOI - auto rotation overshoots means MOI too low, sluggish means too high
- [ ] Run at least one auto routine end-to-end and verify tracking

### Wheel COF Measurement (optional)

- [ ] Measure actual coefficient of friction on competition carpet
- [ ] Update kWheelCoef in Constants.DriveConstants, Drive.java, and settings.json

---

## Priority 2 - Turret

> Motor: SparkMax + NEO (27:1), YAMS Pivot + MAXMotion
> Current: kP=3.0, kD=0, kS=0, kV=1.0, kA=0.05, Profile: 1000 deg/s, 7200 deg/s2
> Time budget: 30-60 minutes

- [ ] Enable Live Tuning via Elastic (SmartDashboard then Turret then Commands then Live Tuning)
- [ ] Verify kS - find minimum voltage to start moving (horizontal, no gravity)
- [ ] Tune kV - match Position vs Reference slopes during cruise
- [ ] Tune kA - match curved accel/decel portions (start 0.001, increase)
- [ ] Tune kP - command 0 deg to 90 deg, settle under 200ms
- [ ] Add kD if overshoot (start 0.05)
- [ ] Test profile limits - increase velocity toward approx 1200 deg/s if stable
- [ ] Verify turret tracks hub target accurately in teleop
- [ ] Verify turret respects plus/minus 190 deg soft limits (no hard-stop impacts)
- [ ] Record final gains: kP=___ kD=___ kS=___ kV=___ kA=___

---

## Priority 3 - Flywheel

> Motors: 2x TalonFX Kraken X60 (1:1), YAMS FlyWheel (velocity)
> Current: kP=0.2, kD=0, kS=0.35, kV=0.12, kA=0
> Time budget: 20-30 minutes

- [ ] Tune kS - ramp voltage until flywheel barely spins (skip in sim)
- [ ] Tune kV - set target velocity (kP=0), adjust until steady-state matches at multiple RPMs (1000, 2000, 3000, 4000)
- [ ] Tune kP - step velocity command, reach setpoint in approx 0.5s (try 0.5-1.0 if current 0.2 is slow)
- [ ] (Optional) Tune kA - for faster spin-up
- [ ] Verify kVelocityTolerance (30 RPM) - tighten if shots inconsistent
- [ ] Test recovery after a shot (flywheel dip and re-spin)
- [ ] Record final gains: kP=___ kS=___ kV=___ kA=___

---

## Priority 4 - Hood

> Motor: TalonFX Kraken X44 (30:1), YAMS Pivot + MotionMagic
> Current: kP=300, kD=0, kS=0.45, kV=3.0, kA=0, Profile: 270 deg/s, 270 deg/s2
> Time budget: 20-30 minutes

- [ ] Verify kS - smooth from standstill (slow sweep)
- [ ] Verify kV - Position matches Reference in Tuner X during cruise
- [ ] Verify kP - command 0 deg to 15 deg to 0 deg, settle under 100ms
- [ ] Add kD only if audible chatter (low inertia amplifies noise)
- [ ] Test profile limits - increase velocity if shot transitions feel slow
- [ ] Verify hood tracks LUT angles at multiple distances
- [ ] Record final gains: kP=___ kD=___ kS=___ kV=___ kA=___

---

## Priority 5 - Intake Pivot

> Motors: 2x SparkFlex + NEO Vortex (30:1), YAMS Arm + ExponentialProfile
> Current: kP=0, kG=0.21, kS=0.11, kV=0, kA=0, Profile: 90 deg/s, 90 deg/s2
> **Profile type: Exponential** — YAMS runs ExponentialProfile on RIO-side Notifier thread for SparkFlex.
> kV and kA here serve double duty: feedforward gains AND exponential profile constraints (max velocity = 12V / kV, time constant = kA / kV).
> CRITICAL: Feedforward is COMMENTED OUT in IntakePivotSubsystem.java. PID is all zeros. Pivot currently has NO active control.
> Time budget: 30-60 minutes

### Pre-requisite: Enable Feedforward

- [ ] Uncomment .withFeedforward(armFeedforward()) in IntakePivotSubsystem.java (line 122)
- [ ] Uncomment .withSimFeedforward(armFeedforwardSim()) (line 123)
- [ ] Deploy and verify no errors

### Tune Gains

- [ ] Tune kG - hold arm horizontal, increase until it holds position (YAMS applies kG times cos(angle) automatically)
- [ ] Tune kS - slow constant-velocity sweep, record breakthrough voltage
- [ ] Tune kV - kV = (voltage - kG*cos(theta) - kS) / velocity
- [ ] Tune kP - command stow (125 deg) to deploy (0 deg), settle under 0.3s (start at 1.0)
- [ ] Tune kD - significant inertia, expect to need some (start 0.05)
- [ ] Increase profile limits - 90 deg/s is very conservative; try 180-270 deg/s after PID is stable
- [ ] Tighten soft limits - currently 0-360 deg (full revolution!), set to actual mechanical range

### Verify Behavior

- [ ] Verify deploy command moves arm to 0 deg
- [ ] Verify stow command moves arm to 125 deg
- [ ] Verify arm holds position when bumped
- [ ] Verify agitate command works (oscillates while intaking)
- [ ] Record final gains: kP=___ kD=___ kG=___ kS=___ kV=___ kA=___

---

## Priority 6 - Spindexer / Kicker / Intake Rollers

> Time budget: 15-30 minutes total (these are simple velocity mechanisms)

### Spindexer

> Motor: SparkMax + NEO (5:1), Current: kP=0.02, kS=0.25, kV=0.6

- [ ] Verify kS/kV - balls feed consistently at 500 RPM
- [ ] Increase kP if balls feed inconsistently (0.02 is very low)
- [ ] Record final gains: kP=___ kS=___ kV=___

### Kicker

> Motor: SparkFlex + NEO Vortex, Current: kP=0, kS=0.25, kV=0.12 (FF-only)

- [ ] Verify kS/kV - consistent kick speed at 2500 RPM
- [ ] Add kP (0.05-0.1) only if kicker stalls on ball contact
- [ ] Record final gains: kP=___ kS=___ kV=___

### Intake Rollers

> Motor: TalonFX Kraken X60 (2:1), Current: kP=0.5, kS=0.39, kV=0.24

- [ ] Verify kS/kV - rollers spin at 2000 RPM
- [ ] Test ball intake - rollers slow too much on contact then increase kP; buzzes then decrease
- [ ] Record final gains: kP=___ kS=___ kV=___

---

## Priority 7 - Shooter Integration (LUT Calibration)

> Time budget: 1-2 hours (requires balls + hub access)

### Two-Point Flywheel Model Verification

- [ ] Measure RPM needed at closest distance (approx 61 in / 1.55m) then compare to kRpmAtMin (2500)
- [ ] Measure RPM needed at farthest distance (approx 291 in / 7.4m) then compare to kRpmAtMax (4300)
- [ ] Update ShooterConstants.ShooterModel if needed

### Hood Angle LUT Calibration

> Current LUT has 6 rows - target 10-12 rows with approx 0.5m spacing

- [ ] Use testShoot command (tuning mode, A button)
- [ ] At each distance: adjust Shooter/HoodAngleDeg until shots land in hub
- [ ] Record distance + hood angle for each position
- [ ] Update ShooterLookupTables.java HUB array with new rows
- [ ] Repeat for PASS table if time permits
- [ ] Verify RPM fudge factor (Shooter/RPMFudgePercent) then adjust if all shots consistently short/long

### Shot Distances to Test

- [ ] Row 1: 1.25m (approx 49 in) - Hood angle: ___
- [ ] Row 2: 1.55m (approx 61 in) - Hood angle: 0 deg (existing, verify)
- [ ] Row 3: 2.25m (approx 89 in) - Hood angle: ___
- [ ] Row 4: 2.83m (approx 111 in) - Hood angle: 3.1 deg (existing, verify)
- [ ] Row 5: 3.50m (approx 138 in) - Hood angle: ___
- [ ] Row 6: 4.11m (approx 162 in) - Hood angle: 7.47 deg (existing, verify)
- [ ] Row 7: 4.75m (approx 187 in) - Hood angle: ___
- [ ] Row 8: 5.25m (approx 207 in) - Hood angle: ___
- [ ] Row 9: 5.80m (approx 228 in) - Hood angle: 10.8 deg (existing, verify)
- [ ] Row 10: 6.25m (approx 246 in) - Hood angle: ___
- [ ] Row 11: 6.74m (approx 265 in) - Hood angle: 12.65 deg (existing, verify)
- [ ] Row 12: 7.40m (approx 291 in) - Hood angle: 12.8 deg (existing, verify)

---

## Priority 8 - Vision and Pigeon

### Pigeon IMU Calibration

- [ ] Inspect Pigeon 2 mounting - which way does the arrow point? Flat or flipped?
- [ ] Configure MountPose in Tuner X if needed (Yaw/Roll/Pitch)
- [ ] Let robot sit 2-3 minutes after power-on (auto-calibrates)
- [ ] Verify: face known direction, rotate 90 deg left then reads +90 deg, right then reads -90 deg
- [ ] Zero yaw before match (Start button)

### PhotonVision Camera Tuning

- [ ] Verify all 4 cameras visible at http://photonvision.local:5800
- [ ] Set resolution to 1280x960 (recommended)
- [ ] Adjust exposure (3000-5000 us) and gain (5 for color, 10-15 for mono)
- [ ] Verify LEDs OFF (glare washes out tags)
- [ ] Test: place AprilTag at 3m then verify detection + estimated pose within approx 5cm
- [ ] Test rapid rotation - reduce exposure if tags blur
- [ ] At competition: adjust exposure/gain during practice matches for arena lighting

---

## Pre-Competition Deploy Checklist

> Do these AFTER all tuning is complete, BEFORE competition matches.

- [ ] Set Constants.tuningMode = false (switches telemetry to MID, disables live tuning)
- [ ] Verify Constants.robot = RobotType.COMPBOT
- [ ] Re-comment setupSysid() in RobotContainer (if not already)
- [ ] Update Constants.LEDsConstants.kNumLEDs to actual strip length
- [ ] Uncomment LEDSubsystem instantiation in RobotContainer (if LEDs are wired)
- [ ] Verify all auto routines appear in dashboard chooser
- [ ] Test selected auto end-to-end on practice field
- [ ] Lock Elastic dashboard widgets (right-click then Lock All)
- [ ] Full battery, verify low-battery alert triggers at 11.0V
- [ ] Run gradlew build - must be BUILD SUCCESSFUL with zero warnings
- [ ] Deploy to robot, verify code starts cleanly
- [ ] Verify RobotMapValidator reports no duplicate IDs in Driver Station

---

## Gain Recording Sheet

> Fill in as you tune. Copy final values back into code.

| Mechanism | kP | kI | kD | kS | kV | kA | kG | Profile Type | Profile Vel | Profile Accel |
|-----------|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:------------:|:-----------:|:-------------:|
| Drive Steer | 100 | 0 | 0.5 | 0.1 | 2.48 | 0 | - | Exponential | Expo | Expo |
| Drive Drive | ___ | 0 | 0 | ___ | ___ | - | - | None | - | - |
| Turret | ___ | 0 | ___ | ___ | ___ | ___ | - | Trapezoidal | ___deg/s | ___deg/s2 |
| Hood | ___ | 0 | ___ | ___ | ___ | ___ | - | Trapezoidal | ___deg/s | ___deg/s2 |
| Flywheel | ___ | 0 | 0 | ___ | ___ | ___ | - | None | - | - |
| Kicker | ___ | 0 | 0 | ___ | ___ | - | - | None | - | - |
| Spindexer | ___ | 0 | 0 | ___ | ___ | - | - | None | - | - |
| Intake Rollers | ___ | 0 | 0 | ___ | ___ | - | - | None | - | - |
| Intake Pivot | ___ | 0 | ___ | ___ | ___ | ___ | ___ | Exponential | ___deg/s | ___deg/s2 |

### Physical Measurements

| Parameter | Nominal | Measured | Updated In Code? |
|-----------|:-------:|:-------:|:----------------:|
| Robot mass (kg) | 63.503 | ___ | [ ] |
| Wheel radius (in) | 1.80 | ___ | [ ] |
| Max speed (m/s) | 6.02 | ___ | [ ] |
| Slip current (A) | 120 | ___ | [ ] |
| Wheel COF | 1.2 | ___ | [ ] |
| Robot MOI (kg*m2) | 6.883 | ___ | [ ] |
