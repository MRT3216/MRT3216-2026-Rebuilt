# Test Mode Tuning & Shooter Calibration

## Overview

Interactive tuning controls are enabled at runtime by entering Driver Station's Test mode. The code that enables the test/tuning bindings lives in `RobotContainer.enableTestBindings()` and is invoked from `Robot.testInit()`.

## How to Enable Tuning Controls
- Open WPILib Driver Station and switch to "Test" mode.
- Robot lifecycle will call `Robot.testInit()` and enable interactive bindings.
- Simulation (`Mode.SIM`) keeps a subset of test bindings active for development.
- Robot hardware profile (COMPBOT vs SIMBOT) is selected at compile time via `Constants.robot`.

## Shooter Calibration Workflow

### Prerequisites
- Driver Station connected, robot enabled in **Test** mode
- Dashboard (AdvantageScope/Elastic) viewing `TestShoot/*` and `Tuning/*` keys
- Balls ready to feed

### Dashboard Tunables
- `Shooter/FlywheelRPM`: Flywheel RPM override. Leave at default for auto-RPM from the two-point model. Change to manually set RPM.
- `Shooter/HoodAngleDeg`: Hood angle in degrees. Adjust until shots land in the hub.

### NetworkTables Telemetry (logged while `testShoot` is active)
- `distanceMeters`: Turret-to-hub distance in meters
- `distanceInches`: Same distance in inches
- `effectiveRPM`: The RPM actually being commanded (auto or override)
- `modelRPM`: What the two-point model would command at this distance
- `rpmOverrideActive`: `true` when `Shooter/FlywheelRPM` differs from its default

## Notes & Troubleshooting
- If you don't see test bindings activate, ensure Driver Station is connected and in Test mode.
- For programmatic activation (integration tests), call `robotContainer.enableTestBindings()` from a lifecycle-safe location.

## Contact
If you want further cleanup or clarification, say so and I'll prepare it.