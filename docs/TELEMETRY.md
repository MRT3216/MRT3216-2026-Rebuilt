Telemetry convention
====================

This document describes the project's convention for CTRE Phoenix `StatusSignal<?>` telemetry
and AdvantageKit / YAMS inputs.

Rules
-----
- Only subsystems that own CTRE Phoenix devices (e.g. `TalonFX`) and declare `StatusSignal<?>`
  fields should call `BaseStatusSignal.setUpdateFrequencyForAll(...)` in the constructor and
  refresh those signals in `updateInputs()`.
- For those subsystems, call `PhoenixUtil.refresh(...)` at the top of `updateInputs()` to ensure
  the sampled Phoenix values are up-to-date before reading them.
- Subsystems using YAMS `SmartMotorController` wrappers (Spark/REV) should populate AdvantageKit
  input fields via the wrapper getters (e.g. `motor.getVoltage()`, `pivot.getAngle()`) and should
  not declare Phoenix `StatusSignal<?>` fields or call `PhoenixUtil.refresh(...)`.

Note: YAMS/REV-wrapped devices (for example a SparkFlex wrapped by `SparkWrapper`) do not use
CTRE Phoenix `StatusSignal<?>` telemetry. Their telemetry should be sourced from the YAMS
wrapper getters and the mechanism's `updateTelemetry()` methods. Adding Phoenix refresh calls or
`StatusSignal<?>` fields for non-CTRE devices is unnecessary and may confuse future maintainers.

Best practices
--------------
- Use `Constants.CommsConstants.DEFAULT_TELEMETRY_HZ` when calling
  `BaseStatusSignal.setUpdateFrequencyForAll(...)` to keep a centralized telemetry rate.
- Disable unused signals with `.setUpdateFrequency(0)` to reduce CAN bandwidth usage.
- Keep `PhoenixUtil` as the single helper for refreshes to make future changes easier.

Example
-------
In a TalonFX-based subsystem:

```java
// constructor
BaseStatusSignal.setUpdateFrequencyForAll(Constants.CommsConstants.DEFAULT_TELEMETRY_HZ,
    positionSignal, referenceSignal);

// updateInputs()
PhoenixUtil.refresh(positionSignal, referenceSignal);
// then read wrapper/getters and populate AdvantageKit inputs
```
