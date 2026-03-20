# Test Mode Tuning & Shooter Calibration

## Overview

Interactive tuning controls are enabled by setting `Constants.tuningMode = true` in `Constants.java`, or by toggling the `/Tuning/tuningMode` key from the dashboard (e.g., Elastic). All tuning/test bindings are gated on this flag—Driver Station "Test" mode is not used.

## How to Enable Tuning Controls
- Set `Constants.tuningMode = true` in `Constants.java` **or** toggle `/Tuning/tuningMode` from the dashboard to enable tuning controls.
- Simulation (`Mode.SIM`) keeps a subset of test bindings active for development.
- Robot hardware profile (COMPBOT vs SIMBOT) is selected at compile time via `Constants.robot`.

## Shooter Calibration Workflow

### Prerequisites
- Driver Station connected, robot enabled, and `Constants.tuningMode` set to `true` (or `/Tuning/tuningMode` set true from dashboard)
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
- If you don't see test bindings activate, ensure Driver Station is connected and `Constants.tuningMode` (or `/Tuning/tuningMode`) is set to `true`.
- For programmatic activation (integration tests), ensure `Constants.tuningMode` is set to `true` before robot code starts.

## Contact
If you want further cleanup or clarification, say so and I'll prepare it.
