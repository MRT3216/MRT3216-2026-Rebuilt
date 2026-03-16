Test-mode tuning
=================

Overview
--------
This project no longer uses a compile-time `Constants.tuningMode` boolean. Interactive "tuning"
controls are enabled at runtime by entering Driver Station's Test mode. The code that enables the
test/tuning bindings lives in `RobotContainer.enableTestBindings()` and is invoked from
`Robot.testInit()`.

Quick facts
-----------
- To enable tuning controls on a physical robot, open the WPILib Driver Station and switch to
  "Test" mode. The robot lifecycle will call `Robot.testInit()` and the container will enable the
  interactive bindings.
- Simulation (`Mode.SIM`) keeps a subset of the test bindings active so developers can exercise
  the same controls during simulation.
- Robot hardware profile (COMPBOT vs SIMBOT) is selected at compile time via
  `Constants.robot` (in `src/main/java/frc/robot/constants/Constants.java`). Change this value and
  rebuild if you need a different hardware profile.

Where to look in the code
-------------------------
- `src/main/java/frc/robot/constants/Constants.java`
  - Use `Constants.getMode()` to determine runtime mode (REAL, SIM, REPLAY).
  - Do not rely on any compile-time "tuningMode" boolean — it has been removed.
- `src/main/java/frc/robot/RobotContainer.java`
  - `enableTestBindings()` installs the runtime-only tuning/test bindings (idempotent).
  - `configureButtonBindings()` installs mode-specific bindings for SIM/REAL.
- `src/main/java/frc/robot/Robot.java`
  - `testInit()` calls into the container to enable the runtime test bindings.

Build and run
-------------
Build the project with Gradle (Windows PowerShell):

```powershell
.\gradlew.bat build
```

If you want the simulator profile, edit `Constants.robot` and set it to `RobotType.SIMBOT`, then
rebuild.

Notes and troubleshooting
-------------------------
- If you don't see test bindings activate on a real robot, ensure the Driver Station is connected
  and in Test mode. The robot must be enabled for the controls to become active.
- If you need a programmatic way to enable test bindings for development without the Driver
  Station (for example in an integration test), call `robotContainer.enableTestBindings()` from a
  lifecycle-safe location in your code (be careful to avoid calling it during static initialization).

Contact
-------
If you want me to open a PR with this doc and any other small cleanup (remove remaining
"tuning" references in docs), say "open PR" and I'll prepare it.
