# Technical Reference

## Telemetry Convention

This project uses CTRE Phoenix `StatusSignal<?>` telemetry and AdvantageKit / YAMS inputs.

- Only subsystems with CTRE Phoenix devices should call `BaseStatusSignal.setUpdateFrequencyForAll(...)` and refresh signals in `updateInputs()`.
- Use `PhoenixUtil.refresh(...)` at the top of `updateInputs()` for up-to-date values.
- YAMS `SmartMotorController` wrappers (Spark/REV) populate AdvantageKit input fields via wrapper getters and do not use Phoenix `StatusSignal<?>`.
- Use `Constants.CommsConstants.DEFAULT_TELEMETRY_HZ` for centralized telemetry rate.
- Disable unused signals with `.setUpdateFrequency(0)` to reduce CAN bandwidth.
- Keep `PhoenixUtil` as the single helper for refreshes.

### Example
```java
// constructor
BaseStatusSignal.setUpdateFrequencyForAll(Constants.CommsConstants.DEFAULT_TELEMETRY_HZ,
    positionSignal, referenceSignal);

// updateInputs()
PhoenixUtil.refresh(positionSignal, referenceSignal);
```

## YAMS Mechanism Abstractions

- YAMS provides `Arm`, `ArmConfig`, and `SmartMotorController` abstractions used by the hood subsystem.
- Includes simulation helpers (`simIterate()`), telemetry integration with AdvantageKit, and vendor wrapper implementations (e.g., `TalonFXWrapper`).
- Command-based best-practice guides (BoVLB / WPILib / ChiefDelphi) describe the "command factory + triggers" pattern used across this repo.

### How to Install YAMS via WPILib Vendordep
1. Open Command Palette (Ctrl+Shift+P).
2. Select: `WPILib: Manage Vendor Libraries` -> `Install new library (online or offline)` -> `Online`.
3. Paste the vendordep URL:
   https://yet-another-software-suite.github.io/YAMS/yams.json
4. Finish the install.

### License Note
YAMS is licensed under LGPL-3.0. Review the YAMS LICENSE before redistributing modified copies.