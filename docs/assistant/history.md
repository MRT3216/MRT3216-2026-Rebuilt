# Assistant Conversation History

This file is the single ongoing archive of all assistant conversation sessions. Each session is appended chronologically. Committed to GitHub so any machine can pick up context.

---

## Session: 2026-02-27 — YAMS APIs, Command Patterns, ShooterSystem Refactor

Branch: mechanisms

Conversation (user and assistant messages only, cleaned and organized):

1) User: Asked about YAMS setSpeed semantics and whether to prefer a YAMS-first, command-returning API vs imperative setMechanismVelocitySetpoint.

Assistant: Recommended YAMS-first approach: expose command-returning helpers like `setVelocity(AngularVelocity)` and `setVelocity(Supplier<AngularVelocity>)` and keep short imperative helpers private for special uses. Suggested stop semantics with two forms: long-hold `stopHold()` (closed-loop zero while scheduled) and one-shot `stopNow()` (imperative, non-blocking for sequences).

2) User: Shared repository files and asked for refactor suggestions.

Assistant: Reviewed `FlywheelSubsystem`, `KickerSubsystem`, `SpindexerSubsystem`, `IntakeRollersSubsystem`, `ShooterSystem`, and `RobotContainer`. Proposed standardized public APIs and stop semantics; recommended `applySetpoint(...)` be private and added a `followTarget(...)` re-applier for live supplier updates when YAMS's supplier-based Command did not re-evaluate live.

3) User: Asked to implement changes. Assistant iteratively applied edits across the repo to:
- Add command-returning `setVelocity(...)` overloads.
- Add `stopNow()` and `stopHold()` semantics.
- Make `applySetpoint(...)` private.
- Add `followTarget(...)` to FlywheelSubsystem to reapply supplier values each tick for live tuning.

4) Assistant: Updated `ShooterSystem` to own an `AtomicReference<AngularVelocity> flywheelTarget` for adjustable tuning, added `bumpFlywheelTarget(...)`, `bumpFlywheelUp/Down(...)`, `startShootingWithAdjustableTarget()` (uses `followTarget(...)`), and refactored `aimAndShoot(...)` to use `HybridTurretUtil.computeMovingShot(...)`.

5) Assistant: Updated `RobotContainer` button bindings:
- Made Right Trigger `whileTrue(...)` for shooting in TUNING and REAL modes.
- Wired A/B to bump up/down temporarily.
- Replaced problematic B `.whileFalse(flywheel.stopHold())` (which scheduled a stop when B was released) with one-shot bump commands so bumps don't interrupt running pipelines.

6) User: Reported that pressing B while holding right trigger caused turret to stop and flywheel wouldn't stop on trigger release.

Assistant: Investigated and fixed conflicts by rebinding A/B and later setting flywheel default command to `flywheel.stopHold()` so releasing triggers returns flywheel to zero. Ran Gradle builds to verify; fixed formatting and compile issues discovered during iteration.

7) Assistant: Performed small cleanup: added `withName(...)` to several composed commands for better runtime logs (e.g., `FeedAndShoot`, `StartShootingAdjustable`, `AimAndShoot`, `StopShooting`), and named `stopHold()` commands like `FlywheelStopHold`, `SpindexerStopHold`, `KickerStopHold`, `IntakeRollersStopHold` for clearer Scheduler traces.

8) User: Asked for small Javadoc/comment polishing. Assistant: Converted inline comments into concise Javadoc for public methods: `bumpFlywheelUp`, `bumpFlywheelDown`, `startShootingWithAdjustableTarget`, and `stopShooting`.

9) User: Asked whether subsystems are still organized and region markers intact. Assistant: Verified `// region` / `// endregion` markers exist across subsystems in the expected order.

10) User: Asked how to back up the chat to GitHub. Assistant: Gave two options — add timestamped markdown into `docs/` and commit to the repo (recommended), or create a private gist via `gh gist create` (requires GitHub CLI). Provided PowerShell commands and offered to commit a backup file.

11) User: Asked assistant to commit a timestamped backup to `mechanisms`. Assistant: Created `docs/assistant/history-2026-02-27.md`, committed and pushed it to `mechanisms`.

12) User: Wanted an ongoing single file (append-only) and the entire chat. Assistant: Suggested creating `docs/assistant/history.md` as the single-file archive and asked whether to include raw full transcript or user/assistant only and whether to redact secrets. User asked to include everything but remove unnecessary items. Assistant created `docs/assistant/profile.md` (starter prompt + conventions) and then created the ongoing `docs/assistant/history.md` with a cleaned transcript appended (this file).

13) Throughout: Assistant ran multiple Gradle builds to verify changes; initially encountered formatting and a Windows DLL file-lock due to simulator running; user stopped the simulator and builds succeeded.

14) Final state & notable files modified (high-level):
- src/main/java/frc/robot/subsystems/shooter/FlywheelSubsystem.java — add followTarget, stopHold/stopNow, getAppliedSetpoint; private applySetpoint.
- src/main/java/frc/robot/subsystems/shooter/KickerSubsystem.java — command-returning APIs, stopHold/stopNow, Javadoc.
- src/main/java/frc/robot/subsystems/shooter/SpindexerSubsystem.java — same pattern as kicker.
- src/main/java/frc/robot/subsystems/intake/IntakeRollersSubsystem.java — same pattern; Phoenix/TalonFX telemetry refreshes.
- src/main/java/frc/robot/systems/ShooterSystem.java — flywheelTarget AtomicReference, bump commands, startShootingWithAdjustableTarget, aimAndShoot refactor, named composed commands, Javadoc.
- src/main/java/frc/robot/RobotContainer.java — cleaned bindings: replaced problematic B whileFalse stop; A/B rebound to bump commands; set flywheel default command to `stopHold()`; right trigger set to whileTrue.
- docs/assistant/history-2026-02-27.md — timestamped backup
- docs/assistant/profile.md — starter prompt and distilled conventions
- docs/assistant/history.md — this ongoing archive (appended now)

15) Tests & verification performed: multiple `./gradlew.bat build` runs after edits; final build successful.

---

## Session: 2026-03-09 — atSpeed Trigger, Misc Refinements

Branch: mechanisms

### Summary

Short session focused on adding an `atSpeed()` method and tolerance constant to FlywheelSubsystem.

### Conversation

1) User: Asked to make `atSpeed()` a method in FlywheelSubsystem with a constant for the error margin.

2) Assistant: Recommended an `AngularVelocity`-typed `kVelocityTolerance` constant in `ShooterConstants.FlywheelConstants` and a `public boolean atSpeed()` method on `FlywheelSubsystem` that compares `Math.abs(measured - setpoint) <= tolerance` using RPM units against the applied setpoint.

3) User: Confirmed RPM tolerance comparing to applied setpoint. Assistant implemented the changes and verified with a build.

### Files Modified
- `ShooterConstants.java` — added `kVelocityTolerance` (now `kVelocityTolerance = RPM.of(30)` in FlywheelConstants).
- `FlywheelSubsystem.java` — added `atSpeed()` boolean method.

*(Session transcript was originally auto-redacted; content above reconstructed from the redacted copy and current codebase state.)*

---

## Session: 2026-03-21 — Constants Reorganization & Cleanup

Branch: Claude

### Summary

Multi-part session focused on reorganizing constants, cleaning up documentation, and auditing PID/FF gains.

### Conversation

1) User: Asked if `Dimensions.java` was used anywhere. Assistant: Searched imports across codebase, confirmed it was completely unused. User asked to delete it.

2) User: Asked to reorganize the `constants/` folder. Assistant: Proposed plan — move subsystem-specific constants into subsystem packages, keep global constants (Constants.java, FieldConstants.java, RobotMap.java) in `constants/`, create lookup tables subfolder. User asked for the recommended plan.

3) User: Noted `TelemetryKeys.java` seemed excessive (each key only used by one subsystem). Assistant: Analyzed usage, confirmed each key was only referenced by its own subsystem. Recommended inlining keys as `private static final String` in each subsystem file.

4) User: Approved full reorganization plan. Assistant executed:
   - Inlined telemetry keys into 7 subsystem files (FlywheelSubsystem, KickerSubsystem, SpindexerSubsystem, HoodSubsystem, TurretSubsystem, IntakeRollersSubsystem, IntakePivotSubsystem).
   - Deleted `TelemetryKeys.java`.
   - Moved `ShooterConstants.java` from `constants/` to `subsystems/shooter/` (updated package).
   - Moved `ShooterLookupTables.java` from `constants/` to `subsystems/shooter/`.
   - Moved `IntakeConstants.java` from `constants/` to `subsystems/intake/`.
   - Updated all imports across the codebase.
   - Removed stale "Telemetry keys are centralized" comments.
   - Build successful.

5) User: Asked to make constants consistently organized across subsystems. Assistant: Analyzed both constants files, identified inconsistencies:
   - Section headings differed across inner classes.
   - PID and FF were sometimes combined, sometimes separate.
   - Simulation overrides had inconsistent ordering (FF-first vs PID-first).
   - `kMotorInverted` placement varied.
   - Kicker used bare `0` instead of `0.0`.
   - FF factories positioned inconsistently relative to limits/tunables.

6) Assistant: Rewrote both files with canonical section ordering:
   - **Velocity subsystems** (Flywheel, Spindexer, Kicker, Rollers): Mechanical → Motor wiring → PID → Feedforward → Simulation overrides (PID then FF) → FF factories → Soft limits → Targets/tunables.
   - **Positional subsystems** (Hood, Turret, Pivot): Mechanical → Motor wiring → PID → Feedforward → Motion profile → Simulation overrides (PID then FF) → FF factories → Hard limits → Soft limits → Presets/tunables.
   - Added section divider comments (`// -----`) for each inner class.
   - Build successful. Committed.

7) User: Asked for PID/FF audit without making changes. Assistant: Reviewed all 8 subsystem gain sets and flagged:
   - 🔴 **Intake Pivot**: kP=0.0, kV=0.0, kA=0.0 — no closed-loop control at all, only gravity comp (kG=0.21, kS=0.11). Sim overrides all zeroed.
   - 🟡 **Kicker**: kP=0.0 — pure feedforward, no proportional correction.
   - All other gains looked reasonable given their gearing ratios and mechanism types.
   - Hood kP=300.0 explained by tiny inertia + 30:1 gearing (normal for CTRE position control).

8) User: Confirmed intake pivot gains were intentional — team abandoned PID and used duty-cycle at first comp. Retune planned for Monday.

9) User: Asked to update and clean up the `docs/assistant/` folder. Assistant: Updated `profile.md` (key files map, stale references, starter prompt, competition status, completion notes) and appended session summary to `history.md`.

### Files Modified
- `docs/assistant/profile.md` — updated key files map, fixed stale `docs/guides/` references, updated starter prompt with constants conventions, added session completion notes, updated competition date.
- `docs/assistant/history.md` — appended this session summary.

### Key Decisions
- Constants live next to their subsystems; global constants stay in `constants/`.
- Telemetry keys are private to each subsystem (no centralized file).
- All inner constant classes follow a canonical section order for consistency.
- Intake pivot zero gains are intentional (retune Monday).

---

End of 2026-03-21 session (part 1: constants reorg).

---

## Session: 2026-03-21 (part 2: knowledge base expansion)

**Goal**: Expand `docs/assistant/profile.md` knowledge base with 10 documentation reference sections covering external tools, APIs, and best practices.

### Steps
1) User: Requested adding 10 documentation reference sections previously suggested by assistant.
2) Assistant: Fetched external documentation from WPILib (flywheel/turret tuning, TrapezoidProfile, ProfiledPIDController, Triggers), CTRE (Tuner X, swerve API), PhotonVision (pose estimation), PathPlanner (named commands, event triggers, auto builder), Elastic Dashboard (GitHub), and AdvantageScope.
3) Distilled all fetched + training knowledge into 10 concise reference sections and inserted into `profile.md` before the "Starter prompt" section.

### Sections Added to profile.md
1. **WPILib PID Tuning Walkthrough** — flywheel velocity vs turret position control strategies, tuning order, applicability to our mechanisms
2. **WPILib TrapezoidProfile & ProfiledPIDController** — API usage, goal vs setpoint, YAMS comparison table
3. **Elastic Dashboard Reference** — setup, features, Java ElasticLib integration, competition tips
4. **CTRE Phoenix 6 Tuner X & Swerve API** — Tuner X features, swerve API 5 core classes, hardware requirements
5. **WPILib Triggers Deep Dive** — getting triggers, binding types table, composing, chaining, debouncing
6. **PhotonVision Constrained Pose Estimation** — all 9 strategies with table, recommended pipeline, constrained SolvePnP heading-assisted
7. **AdvantageScope Log Analysis** — key views table, competition workflow, replay feature
8. **WPILib NetworkTables Best Practices** — performance tips, data types, key organization patterns
9. **PathPlanner Triggers & Named Commands** — AutoBuilder config, named commands, EventTrigger, PointTowardsZoneTrigger, PathPlannerAuto triggers, Java warmup
10. **FRC Battery Management** — testing procedure, health indicators table, current limiting, competition management

### Files Modified
- `docs/assistant/profile.md` — 10 new sections inserted, last-edited footer updated.
- `docs/assistant/history.md` — appended this session summary.

---

End of 2026-03-21 session (part 2).

---

## Session: 2026-03-23 — TuningGuide Expansion & Docs Audit

**Goal**: (1) Expand TuningGuide.md with exponential profile gains section and detailed mechanism subsections, (2) Audit all docs for accuracy and consistency.

### Part 1: TuningGuide.md Expansion (commit a2c19b1)

**Context**: User asked about exponential profiles and TorqueFOC in Q&A. This led to a request to add exponential profile coverage to TuningGuide.md and expand all mechanism sections.

#### Steps
1) User: Q&A about exponential profiles — how kV/kA serve as both feedforward gains and profile constraints, how YAMS handles them on TalonFX (hardware MotionMagicExpoVoltage) vs SparkWrapper (RIO-side Notifier thread running ExponentialProfile).
2) User: Q&A about TorqueFOC — confirmed project uses FOC commutation (via DCMotor.getKrakenX60Foc) but NOT TorqueCurrentFOC control mode.
3) User: Requested TuningGuide.md expansion — exponential profile section + expanded mechanism sections.
4) Assistant: Added new "Exponential Profile Gains" section to the feedforward theory area of TuningGuide.
5) Assistant: Expanded all 7 mechanism sections (Turret, Hood, Flywheel, Kicker, Spindexer, Intake Rollers, Intake Pivot) with: pre-flight checklists, numbered substeps, verification checklists, AdvantageScope signal tables, troubleshooting tables.
6) Assistant: Added Profile Type column to Quick Reference Table.
7) **Issue**: First round of 4 edits was lost between conversation turns (only Kicker expansion persisted). Diagnosed via git diff, redid all edits successfully.
8) Commit a2c19b1 pushed: 694 insertions, 39 deletions.

### Part 2: Docs Folder Audit

**Context**: User asked to "make another pass through the tuning checklist and make sure its up to date and then check all the files in the docs folder."

#### Issues Found & Fixed
1. **README.md** — Missing `TUNING_CHECKLIST.md` from the documentation index. Added it.
2. **TurretAimPipeline.md** — Date header said "2025-03-21" (wrong year). Fixed to "2026-03-23".
3. **TUNING_CHECKLIST.md** — Feedforward line references said "approx line 127-128" but actual commented-out lines are 122-123 in IntakePivotSubsystem.java. Fixed to exact line numbers.
4. **TUNING_CHECKLIST.md** — Gain Recording Sheet was missing a "Profile Type" column (TuningGuide Quick Reference now has one). Added the column with correct profile types for each mechanism.
5. **TUNING_CHECKLIST.md** — Intake Pivot section lacked explanation of exponential profile behavior on SparkFlex. Added clarifying notes about RIO-side Notifier thread and kV/kA double duty.
6. **TestModeTuning.md** — `ShooterLookupTables.java` references didn't include full path after constants reorg. Updated to `subsystems/shooter/ShooterLookupTables.java`.
7. **OperatorGuide.md** — Reviewed, no issues found. Controller bindings match RobotContainer.
8. **TechnicalReference.md** — Reviewed, no issues found. Brief but accurate.
9. **assistant/profile.md** — Updated last-edited date.

### Files Modified
- `docs/README.md` — added TUNING_CHECKLIST.md entry
- `docs/TurretAimPipeline.md` — fixed date
- `docs/TUNING_CHECKLIST.md` — fixed line numbers, added Profile Type column, added exponential profile notes
- `docs/TestModeTuning.md` — fixed ShooterLookupTables.java paths
- `docs/assistant/profile.md` — updated last-edited date
- `docs/assistant/history.md` — appended this session summary

### Part 3: Driver / Operator Control Swap (commit fc1635a)

**Context**: User asked to check which controller owned which subsystem. Found that the driver had shooting and the operator had intake — user confirmed this was backwards.

#### Changes
- **Driver (Xbox controller 0)**: Now owns **intake** — RT toggles intake, LT stops rollers, RB agitate, LB eject.
- **Operator (Xbox controller 1)**: Now owns **shooting** — RT hub shot, LT pass shot, RB manual intake inward, LB clear/unjam.
- **Shift-end rumble**: Now pulses **both** controllers (was only operator).
- Renamed `docs/OperatorGuide.md` → `docs/ControllerGuide.md` and rewrote to document both controllers.
- Updated `docs/README.md` to reference `ControllerGuide.md`.
- Updated `docs/assistant/profile.md` local references.
- Commit fc1635a pushed.

### Part 4: Flywheel Pre-spin Fix (commit 4399411)

**Context**: User ran sim and noticed "the flywheel is just randomly spinning at certain times." Diagnosed as `HubShiftUtil.getShiftedShiftInfo()` returning unpredictable state in sim/tuning mode because there's no FMS game-specific data. The flywheel default command unconditionally used HubShiftUtil to determine pre-spin velocity.

#### Root Cause
`configureDefaultCommands()` had the flywheel pre-spin supplier outside the tuning/competition if-else, so it ran in all modes. In sim with `Constants.tuningMode = true`, HubShiftUtil returned random shift states.

#### Fix
- **Tuning mode branch**: Flywheel default command set to `flywheelSubsystem.stopHold()` with a comment explaining why.
- **Competition branch**: Flywheel pre-spin (with hub-shift-aware velocity supplier) moved into the `else` block where FMS data is available.
- **ControllerGuide.md**: Quick summary updated to note "AUTOMATIC (competition mode only — disabled in tuning mode)" and added "HOOD → returns to 0° when not actively shooting."

#### Also Clarified
- **DS Test mode vs Constants.tuningMode**: YAMS Live Tuning requires DS Test mode, but our robot does NOT use DS Test mode. Our tuning mode is gated on `Constants.tuningMode` (hardcoded boolean). `TestModeTuning.md` explicitly states "Driver Station 'Test' mode is not used."

### Files Modified (Parts 3 & 4)
- `src/main/java/frc/robot/RobotContainer.java` — swapped driver/operator bindings; moved flywheel pre-spin into competition branch
- `docs/ControllerGuide.md` (renamed from OperatorGuide.md) — full rewrite for both controllers; added competition-mode note and hood entry to quick summary
- `docs/README.md` — ControllerGuide.md reference
- `docs/assistant/profile.md` — updated references

---

End of 2026-03-23 session.

---

## Session: 2026-03-23 / 2026-03-24 — TuningDashboard, CRT Calibration, Turret Tuning

**Goal**: (1) Create TuningDashboard Shuffleboard tab, (2) Calibrate CRT encoder offsets on real robot, (3) Diagnose and fix turret PID/FF tuning issue.

### Part 5: TuningDashboard Creation

**Context**: User wanted a Shuffleboard-based tuning dashboard for use during real-robot tuning sessions.

#### Steps
1) Created `src/main/java/frc/robot/util/TuningDashboard.java` — static utility class that builds a "Tuning" Shuffleboard tab with:
   - **Turret Viz**: Mechanism2d (4×4) showing turret angle + setpoint ligament
   - **Turret**: angle, setpoint, volts, current readouts
   - **Hood**: angle, setpoint readouts
   - **Flywheel**: velocity RPM, setpoint RPM, at-speed boolean
   - **LUT Calibration**: measured distance, model RPM prediction, LUT nearest row string
   - **Status**: isAimReady, competitionMode booleans
2) Added `getVelocityRPM()`, `getSetpointRPM()`, `atSpeed()` getters to `FlywheelSubsystem.java`.
3) Wired `TuningDashboard.initialize()` into `RobotContainer` constructor (gated on `tuningMode`).
4) Wired `TuningDashboard.periodic()` into `Robot.java` after `CommandScheduler.run()`.
5) Fixed compile errors: `BuiltInWidgets.kMechanismWidget` doesn't exist (changed to `kTextView`), `Transform3d` to `Transform2d` for `HybridTurretUtil.turretOrigin()`, distance calculation corrected.

**Issue discovered**: Elastic Dashboard does NOT auto-discover Shuffleboard tabs. The Shuffleboard API publishes data to `/Shuffleboard/Tuning/` in NetworkTables, but Elastic needs manual widget layout or SmartDashboard key discovery. Data is accessible in Elastic under NT but requires manual widget setup.

### Part 6: Turret Tuning Command — Clamp Instead of Wrap

**Context**: User wanted the turret tuning command (bumper-based rotation) to use simple clamping at ±90° soft limits instead of the wrap-around logic designed for competition auto-aim.

#### Changes
- `RobotContainer`: Turret tuning default command changed from `turret.setAngle(wrapAngle(...))` to `turret.setAngle(Degrees.of(MathUtil.clamp(current + increment, -90, 90)))`.
- Removed `setAngleRaw` from TurretSubsystem (was a temporary bypass that skipped wrapAngle; not needed with clamping approach).

### Part 7: CRT Encoder Calibration on Real Robot

**Context**: User was at the real robot and needed to calibrate the EasyCRT encoder offsets for absolute turret position bootstrapping.

#### Process
1) Initially added `DutyCycleEncoder` as a persistent field with raw encoder logging in `updateInputs()` — this let the user see raw readings in AdvantageScope.
2) User positioned turret at mechanical zero, read raw encoder values from logs.
3) CRT offsets entered as negative of raw readings: `kCRTEncoder1Offset = Rotations.of(-0.7983)`, `kCRTEncoder2Offset = Rotations.of(-0.86415)`.
4) **CRT solve succeeded!** Status = SOLVED on real robot boot.
5) User later reverted the persistent DutyCycleEncoder field approach (CRT offsets were calibrated; no longer needed persistent access to the raw encoder).

#### Key Learning
- EasyCRT uses `.plus(offset)` on raw readings, so offset = negative of raw reading at mechanical zero.
- AdvantageKit `@AutoLog` serializes `Angle` in **radians** (base SI unit), not degrees — users need to apply unit conversion in AdvantageScope.

### Part 8: Turret PID/FF Unit Analysis & Fix

**Context**: User reported turret was very slow and had ~3° steady-state error even with kP=2, kS=0.2, kV=0.7. Agent went unresponsive during initial investigation (token budget).

#### Root Cause Analysis

**Critical discovery**: The SparkMax's on-board PID and feedforward operate in **mechanism rotations** (not degrees) because YAMS sets:
- `positionConversionFactor = rotorToMechanismRatio = 1/27` → position in mechanism rotations
- `velocityConversionFactor = (1/27)/60` → velocity in mechanism rot/s

Since the turret uses a trapezoid profile, YAMS configures `ControlType.kMAXMotionPositionControl`, meaning **all PID + FF runs on the SparkMax hardware** (not RoboRIO).

REV docs confirm: kV = "Volts per velocity as measured by feedback sensor, **after the conversion factor**"

##### kV was ~5× too low
- kV=0.7 at cruise (1000°/s = 2.778 rot/s): FF = 0.2 + 0.7 × 2.778 = **2.1V** (need ~9.5V)
- Correct kV ≈ **3.42** V/(mech rot/s): FF = 0.2 + 3.42 × 2.778 = **9.7V**
- Calculation: NEO free speed 5676 RPM, at cruise motor RPM = 2.778 × 27 × 60 = 4500, voltage = 4500/473 = 9.51V, kV = 9.51/2.778 = 3.42

##### kP was astronomically too low
- kP=2 means 2V per **full rotation** (360°) of error
- At 3° error: 2 × (3/360) = **0.017V** — essentially zero correction
- Need kP ≈ **100** to get ~0.83V at 3° error, ~2.8V at 10° error

#### Fix Applied
- `kP`: 0.0 → **100.0** (start conservative, increase if needed)
- `kV`: 0.7 → **3.4** (slightly below theoretical 3.42, tune on robot)
- Added detailed unit explanation comments in ShooterConstants.java

### Part 9: Soft/Hard Limit Changes (user-modified on robot)

During real-robot testing, the user also changed:
- Soft limits: ±190° → ±90°
- Hard limits: ±195° → ±95°
- kA: 0.05 → 0.01
- Gearing confirmed at 27:1 (was previously 27, verified correct)

### Files Modified This Session
- `src/main/java/frc/robot/util/TuningDashboard.java` — **NEW**: Shuffleboard tuning tab
- `src/main/java/frc/robot/subsystems/shooter/FlywheelSubsystem.java` — added getVelocityRPM(), getSetpointRPM(), atSpeed() getters
- `src/main/java/frc/robot/Robot.java` — added TuningDashboard.periodic() + import
- `src/main/java/frc/robot/RobotContainer.java` — TuningDashboard.initialize(), turret tuning command clamped (not wrapped), flywheel default in tuning → stopHold()
- `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java` — turret kP=100, kV=3.4, kA=0.01, limits ±90/±95, unit explanation comments
- `src/main/java/frc/robot/subsystems/shooter/TurretSubsystem.java` — CRT offsets calibrated (user also reverted persistent DutyCycleEncoder field)
- `docs/ControllerGuide.md` — competition-mode-only note in quick summary

### Key Technical Findings
1. **SparkMax PID/FF units with YAMS**: When YAMS sets conversion factors, all SparkMax on-board PID/FF operates in mechanism units (rotations, rot/s). kP is Volts/rotation, kV is Volts/(rot/s). This makes raw kP/kV values much larger than you'd expect if thinking in degrees.
2. **MAXMotion with trapezoid profile**: Runs entirely on SparkMax hardware. No RIO-side closed-loop thread (that's only for exponential profiles or LQR).
3. **EasyCRT calibration**: Offset = negative of raw reading at mechanical zero. The .plus(offset) call handles the correction.
4. **Elastic vs Shuffleboard**: Elastic doesn't auto-discover Shuffleboard API tabs. Need either manual Elastic layout or switch to SmartDashboard.putNumber() approach.

### Turret Tuning Next Steps (for next session at robot)
1. Deploy with kP=100, kV=3.4, kS=0.2, kA=0.01
2. Test feedforward first: disable kP (set to 0), command a constant velocity, verify turret moves at expected speed
3. Re-enable kP=100, test step response. Increase to 150-200 if sluggish, decrease if oscillating
4. Add kD (start ~1.0) if there's overshoot or oscillation
5. Fine-tune kS if turret has a dead zone at very low speeds

---

End of 2026-03-23/24 session.

---

## Session: 2026-03-25 -- CRT Completion, Gain Tuning, Hybrid Aiming Wiring

Branch: Claude -> merged to main

### CRT Validation (completed)
- CRT fully validated across 7+ positions. Worst-case ErrorRot = 0.008 (tolerance = 0.02)
- Final offsets: Enc1 = -0.821, Enc2 = -0.805
- Pinion teeth: Enc1 = 10T (SparkMax), Enc2 = 13T (RoboRIO PWM DIO)
- CRT retry logic: 100ms settle + 5 retries implemented and working
- Tolerance tightened from 0.05 to 0.02 (commit 190f001)
- Continuous raw encoder logging added then removed (commit 198b1d8)
- pwmEncoder kept as persistent field in TurretSubsystem (not local to attemptCRTSolve)

### Turret Gain Tuning (in progress)
Progression of gains tested on real robot:
- kP=60 -> motor saturation, massive overshoot (15V at 90 deg error)
- kP=3/kV=3.4 -> slow, oscillating
- kP=6/kV=2.4 -> still overshooting
- kP=8/kD=0.3/kV=2.4 -> better small moves, big moves still overshoot
- kP=8/kD=0.5/kV=2.4 -> too slow (kD resists all motion)
- kP=10/kD=0.15/kV=2.4/maxVel=800/maxAccel=3600 -> still overshooting big moves
- kP=15/kD=0.15/kV=1.8/maxVel=800/maxAccel=3600 -> last deployed, untested after snap-to-zero fix

Current constants in code: kP=3, kD=0.3, kV=1.0, kA=0.05, maxVel=720, maxAccel=3600
(Note: the on-robot deployed values may differ from code if tuned via dashboard)

**Key finding**: SparkMax MAXMotion kV needs to be lower than theoretical (3.4) because
the profile cruise velocity feeds too much voltage. kV=1.8-2.4 range worked better.

### Snap-to-Zero Fix
- Problem: turret snapped to 0 deg on every enable because turretAccumulator was initialized to {0.0}
- Fix: Changed to Commands.sequence with runOnce to re-seed turretAccumulator from turretSubsystem.getPosition() on each enable
- This ensures the D-pad tuning command starts from wherever the turret actually is

### Branch Cleanup
- Claude branch merged to main (PR #9)
- Deleted local branches: Claude, Pivot-Arm-Banch, copilot/sub-pr-7
- Deleted remote branches: copilot/sub-pr-7, copilot/sub-pr-9, copilot/sub-pr-9-again
- Only remaining branch: new-mechanism (2 PathPlanner commits)

### Hybrid Aiming Wired in Tuning Mode (commit 18f6e29)
Following docs/HybridAiming.md guide, wired hybrid aiming for tuning mode:

**ShooterSystem.java**: Added hybridAim() method -- aim-only variant of hybridAimAndShoot.
Turret clamped to +/-deadband, hood tracks, telemetry logs, but NO flywheel spin and NO feed.
Safe for testing alignment without firing balls.

**ShooterConstants.java**: kTurretDeadbandDeg changed from 90.0 to 30.0 (+/-30 deg turret clamp)

**RobotContainer.java** tuning mode changes:
- Drive default: joystickDrive -> joystickDriveAimAtTarget (drivetrain heading assist, gated on RT > 0.5)
- RT: testShoot -> hybridAim (aim only, no shoot)
- A: aim (turret-only) -> testShoot (turret 0 deg + flywheel + feed)
- Old A binding commented out for easy reversal
- B/X/Y/bumpers/LT unchanged

### Turret Limits (current in code)
- Hard limits: +/-190 deg
- Soft limits: +/-180 deg
- Hybrid aiming clamp: +/-30 deg (only applies when hybridAim/hybridAimAndShoot active)

### Technical Notes
- VS Code editor tool (replace_string_in_file) edits in-memory buffers but does not always flush to disk.
  When this happens, changes appear in grep_search/read_file but git sees no diff.
  Workaround: use terminal-based Python scripts with pathlib to write files directly.
  This is a session-specific bug that occurs when context gets very large.
- spotlessApply runs before compileJava and reformats all .java files. It reformats but does not revert content.
- PowerShell Set-Content corrupts UTF-8 special characters (replaced em-dashes, degree symbols with garbage).
  Use [System.IO.File]::WriteAllBytes with UTF8 encoding instead, or Python pathlib.

### Next Steps for Next Session
1. **Turret gain tuning**: Resume with kP=15/kV=1.8 gains (or current code values). Small moves OK, big moves still overshoot. Consider:
   - Reducing maxVel further (try 600 deg/s)
   - Increasing kD slightly (0.2-0.3 range)
   - Testing with hybridAim where turret only needs +/-30 deg moves (much safer for tuning)
2. **Test hybrid aiming on robot**: Verify drivetrain heading assist + clamped turret tracking works
3. **Hood tuning**: Hood gains not yet tuned on real robot
4. **Flywheel tuning**: Flywheel gains partially tuned (kP=0.2, kV=0.12 in code)
5. **LUT calibration**: See docs/TestModeTuning.md for the 12-row calibration plan

---

End of 2026-03-25 session.
