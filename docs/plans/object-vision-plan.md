# Execution Plan: Object Vision Library

**Created by**: Claude (chat) on 2026-09-19
**Branch**: `feature/object-vision`
**Repository**: MRT3216/MRT3216-2026-Rebuilt (Java 17, WPILib 2026, AdvantageKit, PhotonLib, PathPlanner)

> This adapts `docs/assistant/plan-format.md` for a greenfield library. The original format says "no judgment calls, stop on any failure". Here the design decisions are made (below), but you may fix compile errors inside a step's own files. Anything outside a step's scope: stop and ask.

## How to run this plan

1. One phase per session. `/clear` between phases. Read only `CLAUDE.md` and this file.
2. Start each phase in plan mode (Shift+Tab). Propose file-level changes and wait for approval.
3. Every phase ends at a **Gate**. Report evidence (test output, sim observations) and stop. Do not begin the next phase.
4. Verification commands: `./gradlew compileJava`, `./gradlew test`, `./gradlew simulateJava` (Windows: `.\gradlew`).
5. Commit once per phase with a descriptive message.

## Context (verified from the repo)

- GradleRIO 2026.2.1, Java 17, PhotonLib v2026.3.2, AdvantageKit 26.0.2, PathplannerLib 2026.1.2, YAMS. JUnit 5 is configured in `build.gradle`; `src/test` does not exist yet.
- Spotless (google-java-format, 4 spaces) runs before every compile.
- `subsystems/vision/*` is the AdvantageKit vision template: `VisionIO` (`@AutoLog` inputs, records), `VisionIOPhotonVision`, `VisionIOPhotonVisionSim`, `Vision`. Four AprilTag cameras: Front, Left, Right, Rear. `Vision` feeds `drive::addVisionMeasurement`.
- `Drive` owns a `SwerveDrivePoseEstimator`, exposes `getPose()`, configures `AutoBuilder`, and sets `Pathfinding.setPathfinder(new LocalADStarAK())`. No calls to `pathfindToPose` were found in `RobotContainer` or `DriveCommands`.
- `DriveCommands` has `joystickDrive`, `joystickDriveAtAngle`, `joystickDriveAimAtTarget` (static factories taking suppliers).
- Driver holds LT for `intakeSystem.dutyCycleIntake()`.
- `RobotContainer` builds three configs: real, SIM, and replay (default). SIM currently uses only Front and Back camera sims because `VisionSystemSim` costs about 8-10 ms per camera (per the comment in the code).
- Conventions: three-layer pattern, Noop IO in every config (never `null`), command factories with `.withName(...)`, `kName` constants with units, no unnamed numeric literals.
- Sim vision currently passes `drive::getPose` (the *estimated* pose) as the sim pose. PhotonVision docs say the sim pose should be separate from the estimated pose. With noiseless `ModuleIOSim` this is acceptable for now; note it in docs.

**Game and hardware facts**
- FUEL: yellow foam ball, 5.91 in (0.150 m) diameter, 504 in the game. Center height when on carpet is about 0.075 m. Expect piles, not isolated balls.
- PhotonVision object detection: NPU boards only (Orange Pi 5 and Rubik Pi 3). Frames are letterboxed to 640x640. Output is a 2D box with class and confidence; there is no 3D pose. Up to 127 targets can be configured. Only quantized 640x640 YOLOv5/v8/v11 models load.
- Cameras on hand: ThriftyCam (monochrome, 1600x1304, USB 3.0) and Arducam OV9782 (color, 1280x800, MJPG, USB 2.0, 70 deg horizontal FOV, no IR-cut filter). Assume the OV9782 is the object detection camera.
- A camera runs one pipeline at a time as far as we know. The object detection camera must be a dedicated fifth camera with its own name. Confirm in PhotonVision docs.

## Goal

Build a reusable, game-agnostic object detection library (`org.mrt3216.objectvision`) that turns PhotonVision object detection results into filtered field-frame target positions, works in WPILib simulation, and drives a driver-assist intake command. Retargeting for a new game piece should be a config change (class ID, target height, thresholds).

## Non-goals

- No changes to the existing AprilTag `Vision` behavior, `Drive`, `generated/TunerConstants.java`, PathPlanner paths, or autos.
- No new vendordeps.
- No orientation estimation (PhotonVision returns 2D boxes only).

## Design decisions (already made; do not revisit)

1. **Location.** The library lives in `libs/objectvision/` with its own `src/main/java` and `src/test/java`, package `org.mrt3216.objectvision`, wired through `sourceSets` in `build.gradle`. Later it can move to its own repo and return as a git submodule at the same path with no code change (add a Spotless exclude then).
2. **Library depends only on** WPILib, AdvantageKit, PhotonLib. It never imports `frc.robot.*`.
3. **Layers.** `CameraGeometry` (pure math) -> `RobotPoseHistory` -> `TargetTracker` -> `TargetSelector` -> `ObjectVision` subsystem. IO follows the repo pattern: `ObjectVisionIO` (`@AutoLog` inputs), `ObjectVisionIOPhotonVision`, `ObjectVisionIOSim`, and a Noop default.
4. **Game-specific data stays in the robot repo**: `frc.robot.subsystems.objectvision.ObjectVisionConstants` (FUEL config, camera name, `robotToCamera`).
5. **Own pose history.** The tracker keeps a `TimeInterpolatableBuffer<Pose2d>` fed from a pose supplier each loop, so `Drive` needs no changes. Detections use `result.getTimestampSeconds()` (FPGA time, no conversion).
6. **Frame-agnostic.** The tracker works in whatever field frame the supplied pose uses. No alliance flipping inside the library.
7. **Performance.** Preallocate lists, no streams, no `Logger` calls inside per-detection loops. Loop overruns have been a problem on this robot.
8. **Sim strategy.** A cheap pure-math sim IO is the default. A PhotonLib-based sim IO is optional (Phase 3b) and only if the API exists in v2026.3.2.
9. **Driver-assist before autonomous pathing.** Autonomous chase is out of scope until the Phase 4 gate is met.

## Verify before coding (Phase 0 output)

Do not assume these; check them against the pinned versions and record answers in the plan's "Findings" section at the bottom.

- [x] Java getters on `PhotonTrackedTarget` for object detection class ID and confidence (C++ docs show `objDetectId` and `objDetectConf`).
- [x] Exact PhotonVision yaw/pitch sign and formula for detections. Read `TargetCalculations` in the PhotonVision repo. Record whether yaw positive is right, pitch positive is up, and whether pitch includes a yaw correction.
- [x] Whether `VisionTargetSim` in v2026.3.2 has the object-detection constructor (`pose, model, classId, confidence`). The current docs show it; the pinned version is unverified.
- [x] FUEL model class index and label (from the PhotonVision jar resources, not from memory).
- [x] `TimeInterpolatableBuffer` API in WPILib 2026.
- [x] `./gradlew test` runs in this environment.

## Phase 0: Setup and verification

**Pre-conditions**: on branch `feature/object-vision`; `./gradlew compileJava` passes.

1. Create branch. Confirm `CLAUDE.md` loads (`/context`).
2. Answer the "Verify before coding" checklist. Findings go at the bottom of this file.
3. Propose the `build.gradle` change (Gate 0):
   ```groovy
   sourceSets {
       main { java { srcDir 'libs/objectvision/src/main/java' } }
       test { java { srcDir 'libs/objectvision/src/test/java' } }
   }
   ```
   Explain that the AutoLog annotation processor applies to the added source directory.

**Gate 0** (Abram approves): the `build.gradle` change. Nothing else proceeds until approved.

## Phase 1: Pure-math core and tests (no robot, no sim)

Create in `libs/objectvision/src/main/java/org/mrt3216/objectvision/`:

- `TargetConfig` (record): `name`, `classId`, `targetCenterHeightMeters`, `minConfidence`, `maxRangeMeters`, `expirySeconds`, `associationGateMeters`.
- `PhotonAngles` (small helper): the single place that converts PhotonVision yaw/pitch to a camera-frame ray. Use the conventions recorded in Phase 0. `CameraGeometry` and the sim must both use it so they cannot disagree.
- `CameraGeometry`:
  `static Optional<Translation2d> groundPointInRobotFrame(Transform3d robotToCamera, double yawRadians, double pitchRadians, double targetCenterHeightMeters)`.
  Method: build the ray in the camera frame, rotate by the camera rotation, intersect with the horizontal plane `z = targetCenterHeightMeters`. Return empty if the ray does not descend toward the plane.
- `RobotPoseHistory`: wraps `TimeInterpolatableBuffer<Pose2d>`; `record(timestamp, pose)` and `Optional<Pose2d> sample(timestamp)`; drop samples older than a named horizon.
- `TrackedTarget` (record) and `TargetTracker`: associate by nearest neighbor within the gate, smooth with an exponential filter, expire by time, cap track count with a named constant, `clear()`. No allocation in `update` beyond preallocated storage.
- `TargetSelector` (functional interface) and `Selectors`: `nearest()`, `nearestInHeading(halfAngle)`, `densestCluster(radiusMeters)`.

**Tests** in `libs/objectvision/src/test/java/...` (JUnit 5):
- Geometry vectors, camera at origin, height 0.5 m, pitched 20 degrees down (`Rotation3d` pitch +20 degrees is down; `-15` is up per the PhotonLib docs), ball center 0.075 m. Assuming pitch positive is up:
  - Target at yaw 0, pitch 0 -> about 1.168 m ahead (0.425 / tan 20 degrees).
  - Target at yaw 0, pitch +10 degrees -> about 2.410 m ahead (0.425 / tan 10 degrees).
  - Ray above the plane -> empty.
  - Symmetry: yaw +a and -a mirror in y.
  Update expected values if Phase 0 finds different conventions, and say why.
- Tracker: association within gate, new track outside gate, expiry, cap behavior, smoothing convergence, determinism.
- Selectors: nearest, heading window, cluster centroid.

**Verification**: `./gradlew test` passes. Prefer `/goal` with that condition.

**Gate 1**: summarize test output and any convention findings that changed the vectors. Stop.

## Phase 2: IO, subsystem, real hardware path (log-only)

In the library:
- `ObjectVisionIO` with `@AutoLog` inputs: `connected`, `DetectionObservation[] detections` (record: `classId`, `confidence`, `yawRadians`, `pitchRadians`, `areaPercent`, `timestampSeconds`). Default no-op `updateInputs`.
- `ObjectVisionIOPhotonVision(String cameraName)`: reads `camera.getAllUnreadResults()`; skips low-confidence or wrong-class detections only in the subsystem, not the IO (the IO reports everything).
- `ObjectVision` (extends `SubsystemBase`): constructor takes `TargetConfig`, `Supplier<Pose2d>`, and `CameraSpec...` (record: `ObjectVisionIO io`, `Transform3d robotToCamera`). Exposes `getTracks()`, `Optional<TrackedTarget> getBest(TargetSelector)`, and `public final Trigger hasTarget`. Logs `ObjectVision/Camera<i>`, `ObjectVision/Tracks`, `ObjectVision/Selected`, `ObjectVision/Summary/HasTarget`, `ObjectVision/Summary/TrackCount`. Disconnected-camera `Alert` like `Vision`.

In the robot repo:
- `frc.robot.subsystems.objectvision.ObjectVisionConstants`: FUEL `TargetConfig` (class ID from Phase 0), camera name, `robotToCameraObject` as a **placeholder marked `// TODO(measure)`**. Named constants for thresholds.
- `RobotContainer`: instantiate `ObjectVision` in every config. Real: `ObjectVisionIOPhotonVision`. Replay: `new ObjectVisionIO() {}`. SIM: Noop until Phase 3.

**Gate 2** (Abram supplies): which side of the robot the intake faces, and the measured camera transform. Phases 2-3 can proceed with the placeholder; Phase 4 cannot.

## Phase 3: Simulation

Goal: run `./gradlew simulateJava`, open AdvantageScope on `localhost`, and see balls, detections, tracks, and the selected target while driving.

In the library:
- `SimCameraModel` (record): horizontal FOV, vertical FOV, max range, yaw/pitch noise standard deviations, dropout probability, latency, frame rate. OV9782 starting point: 70 degrees horizontal FOV. Other values are starting points; tune later.
- `ObjectVisionIOSim`: takes a truth pose supplier, a ball-position supplier (`Supplier<List<Translation3d>>`), the camera transform, and a `SimCameraModel`. Projects each ball into the camera frame, keeps those in view and range, converts to yaw/pitch with `PhotonAngles`, applies seeded noise (`java.util.Random` with a fixed seed), dropout, and latency. No OpenCV, no streams.
- `FieldObjectSource` (interface) with a `StaticObjectSource` implementation: ball list with `add`, `remove`, `removeWithin(Pose2d robot, rectangle)`.

In the robot repo:
- `ObjectVisionSim` helper (SIM branch only): scenarios `SINGLE_BALL`, `LINE_OF_FIVE`, `CLUSTER`, placed relative to the robot pose when loaded, chosen with a `SendableChooser`. Logs `ObjectVisionSim/Truth` as `Pose2d[]`.
- SIM branch of `RobotContainer`: wire `ObjectVisionIOSim` using `drive::getPose` as the truth pose (document the caveat in Context).
- Optional: remove balls when the intake is active and the ball is inside an intake rectangle (wire the condition in `RobotContainer`'s SIM block only; do not add cross-subsystem references to subsystems).

**Tests**: a round-trip test: project a known field ball with the sim math, recover with `CameraGeometry` and the tracker, expect the error to be near zero without noise and within an initial target (for example 0.10 m at 3 m with default noise; adjust with reasoning) with noise.

**Acceptance (run and report)**: `./gradlew simulateJava` runs without loop overruns; in AdvantageScope, tracks match truth as the robot drives past `SINGLE_BALL` and `CLUSTER`; `hasTarget` toggles correctly. Note that the sim GUI is disabled by default in `build.gradle` to support AK replay. To drive with keyboard or a gamepad, tick the Sim GUI extension in the VS Code simulate prompt, and leave it off when doing AK log replay.

**Gate 3**: report numbers and screenshots-worth of evidence (log keys, observed errors). Stop.

### Phase 3b (optional): PhotonLib-based sim IO

~~Only if the Phase 0 check found the object-detection `VisionTargetSim` constructor in v2026.3.2.~~

**Not possible.** Phase 0 verified that `VisionTargetSim` in v2026.3.2 only has fiducial constructors: `(Pose3d, TargetModel)` and `(Pose3d, TargetModel, int)`. No classId/confidence parameters exist. Skip this phase entirely.

## Phase 4: Driver-assist intake command

In `DriveCommands` (static factory, matches `joystickDriveAimAtTarget`):
`joystickDriveAssistedIntake(Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, Supplier<Optional<Translation2d>> targetFieldSupplier)`.

Behavior:
- Translation: identical to `joystickDrive`, including alliance handling.
- Rotation: if a target exists and the driver's rotation input is inside the deadband, use a `ProfiledPIDController` to point the intake at the target (intake-facing side comes from Gate 2). Otherwise pass the driver's rotation through.
- Losing the target or driver override resets the controller; no sudden jump.
- Gains and authority are `LoggedTunableNumber`s. `.withName("JoystickDriveAssistedIntake")`.
- The command takes a plain `Supplier<Optional<Translation2d>>`, not a library type.

**Binding (Gate 4)**: propose a button; do not change the existing LT behavior without approval. Default: no binding until approved. Add it in the driver-controls block in `RobotContainer`, and for every config.

**Verification**: `./gradlew compileJava`, sim run with `LINE_OF_FIVE`, confirm heading assist engages and releases correctly.

## Phase 5: Documentation

- `docs/ObjectVision.md`: architecture, log keys, sim instructions, the caveat about estimated vs. truth pose, and a **retarget checklist** for a new game: class ID, target center height, confidence and expiry thresholds, camera transform, camera calibration, model import steps.
- Add a **model pipeline runbook stub**: capture images, label, train, convert (Orange Pi `.rknn`; Rubik Pi via its own conversion notebook), quantize, import to PhotonVision. Note that PhotonVision converts but does not train, and that Ultralytics YOLO is AGPL-licensed. Leave timings blank for the humans to fill in.
- Add the new files to the Key File Map in `.github/copilot-instructions.md`.

## Human tasks (Claude Code cannot do these)

- Measure and enter the camera transform; calibrate every object detection camera in the PhotonVision UI.
- Bench accuracy test with the real Rubik Pi and OV9782: balls at tape-measured positions, compare against logged tracks. Also compare OV9782 vs ThriftyCam detection range and false positives under venue lighting.
- Model pipeline dry run with timing for each step, on both boards.
- Decide the assist button (Gate 4) and intake-facing side (Gate 2).

## Later (not part of this plan)

- Extract `libs/objectvision` to its own repo and add it back as a submodule (add a Spotless exclude for the path).
- Autonomous chase (`AutoBuilder.pathfindToPose` or direct drive-to-pose) as a `NamedCommand`. It must terminate (timeout or end condition). Requires approval to touch autos.
- WPILib 2027 migration: PhotonVision's 2027 alpha javadocs reference `org.wpilib.*` packages, which suggests an import rename. Expect a mechanical change in January; keep WPILib imports out of unnecessary places.

## Verification (after each phase)

```
./gradlew compileJava
./gradlew test
```
Expected: `BUILD SUCCESSFUL`. For Phases 3-4 also run `./gradlew simulateJava` and report what you observed in AdvantageScope.

## Commit

```
git add -A
git commit -m "feat(objectvision): <phase summary>"
```

## Rollback

`git checkout -- .` for uncommitted work; `git revert <commit>` for a finished phase. Checkpoints (`/rewind`) do not track changes made through shell commands.

## Notes for executor

- Spotless may reformat; that is fine.
- Do not read `docs/assistant/profile.md` or `history.md` in full; grep sections.
- Ask first: vendordeps, folder restructuring (only the approved `libs/objectvision` layout), new Systems, autos.
- Keep `RobotContainer` edits minimal and identical in shape across configs. Every config needs an `ObjectVision` instance (Noop where there is no hardware).
- After two failed attempts on the same problem, stop, summarize what was tried, and ask.

## Session prompts (copy into Claude Code)

- **Phase 0**: "Read CLAUDE.md and docs/plans/object-vision-plan.md. Do Phase 0 only, in plan mode first. Answer the verify checklist against the pinned versions and write findings at the bottom of the plan. Stop at Gate 0."
- **Phase 1**: "Do Phase 1 of the plan. Write the library classes and JUnit tests. Use `/goal` with the condition that `./gradlew test` passes. Then use a subagent to review the diff against the plan, flagging only correctness or requirement gaps."
- **Phase 3**: "Do Phase 3. After compiling, run `./gradlew simulateJava`, and tell me exactly which AdvantageScope log keys I should open to verify tracks vs truth."

## Findings (Phase 0 — verified 2026-09-19)

### 1. PhotonTrackedTarget object detection getters — CONFIRMED (v2026.3.2)
- `getDetectedObjectClassID()` returns `int` (backed by field `objDetectId`)
- `getDetectedObjectConfidence()` returns `float` (backed by field `objDetectConf`)
- Default value is `-1` for both fields when the target is not from object detection

### 2. Yaw/Pitch sign conventions — CONFIRMED (v2026.3.2)
- **Yaw: left = positive, right = negative** (degrees)
- **Pitch: up = positive, down = negative** (degrees)
- Coordinate transform is from OpenCV EDN to WPILib NWU, handled in `OpenCVHelp`
- Implication for `CameraGeometry`: a ball on the ground viewed from a downward-pitched camera will have a **negative** pitch value (target is below the optical axis). The plan's Phase 1 test vectors must account for this — PhotonVision pitch positive is **up**, which matches WPILib convention but is inverted from what some teams expect.
- No yaw correction is applied to pitch (pitch is raw from the bounding box center).

### 3. VisionTargetSim object-detection constructor — DOES NOT EXIST (v2026.3.2)
- Only two constructors available:
  - `VisionTargetSim(Pose3d pose, TargetModel model)`
  - `VisionTargetSim(Pose3d pose, TargetModel model, int fiducialId)`
- No `classId` or `confidence` parameters exist
- **Phase 3b is not possible** with the pinned version. Marked as skipped in the plan above.

### 4. FUEL model class index — NOT DEFINED IN PHOTONLIB
- PhotonVision does not ship game-piece class ID constants
- Class IDs are application-defined by the YOLO model the team trains/imports
- **Use placeholder `0`** (conventional first class in a single-class YOLO model)
- The actual ID will be determined when the team trains or imports a FUEL detection model

### 5. TimeInterpolatableBuffer API — CONFIRMED (WPILib 2026)
- Factory: `TimeInterpolatableBuffer.createBuffer(double historySize)` — for types implementing `Interpolatable<T>`
- `addSample(double time, T value)` — void
- `getSample(double time)` — returns `Optional<T>`
- `clear()` — void
- `getInternalBuffer()` — returns `NavigableMap<Double, T>` (for inspection/debug)
- `Pose2d` implements `Interpolatable<Pose2d>` — fully compatible, no custom interpolator needed
- History auto-cleanup is built-in; no manual eviction required

### 6. `./gradlew test` — FUNCTIONAL (with caveat)
- JUnit 5 (Jupiter 5.10.1) is configured in `build.gradle` with `useJUnitPlatform()`
- `src/test/` directory does not exist yet — `test` task reports NO-SOURCE (not a failure)
- **Pre-existing Spotless issue**: `RemoveUnusedImports` throws `InvocationTargetException`, blocking both `compileJava` and `test` because `compileJava` depends on `spotlessApply`
- Workaround: `./gradlew test -x spotlessApply -x spotlessJava` succeeds
- Code itself compiles cleanly when Spotless is bypassed
- This Spotless issue is on `main` and is unrelated to this work; should be fixed separately

### Pre-existing issue: Spotless failure — FIXED
The Spotless `RemoveUnusedImports` step threw `NoSuchMethodError: JCImport.getQualifiedIdentifier()` due to google-java-format incompatibility with JDK 21. Fixed by bumping Spotless plugin from 6.12.0 to 6.25.0 in `build.gradle`. Both `./gradlew compileJava` and `./gradlew test` now pass cleanly.
