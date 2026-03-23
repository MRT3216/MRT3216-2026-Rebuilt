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

---

## Lookup Table (LUT) Calibration Plan

### Background

The shooter uses a distance-indexed lookup table (`subsystems/shooter/ShooterLookupTables.java`) to map turret-to-hub distance → hood angle + time-of-flight. Flywheel RPM comes from the separate two-point `ShooterModel`, not the LUT.

The current HUB table has **6 rows** with uneven spacing (gaps up to 1.7 m in the critical 4–6 m band). Linear interpolation across large gaps degrades accuracy. The target is **10–12 rows** with ~0.5 m spacing where the hood angle curve changes fastest.

### Recommended Hub Distances (12 rows)

| Row | Distance (m) | Distance (in) | Notes |
|-----|-------------|---------------|-------|
| 1   | 1.25        | ~49"          | Close-range bumper shot |
| 2   | 1.55        | ~61"          | Existing closest point |
| 3   | 2.25        | ~89"          | Fills gap between rows 2→4 |
| 4   | 2.80        | ~110"         | Near existing 2.83 m point |
| 5   | 3.50        | ~138"         | New — fills 2.83→4.11 m gap |
| 6   | 4.10        | ~162"         | Existing midpoint |
| 7   | 4.75        | ~187"         | **New — fills the big 4.1→5.8 m gap** |
| 8   | 5.40        | ~213"         | **New — fills the big gap** |
| 9   | 5.80        | ~228"         | Existing point |
| 10  | 6.40        | ~252"         | Adjusted from existing 6.74 m |
| 11  | 7.00        | ~276"         | New — smooths the far end |
| 12  | 7.40        | ~291"         | Existing max range |

**Spacing rationale:**
- **0.5 m spacing in the 3–6 m band** — most shots are taken here and hood angle changes fastest.
- **0.75 m spacing at extremes** — the curve is flatter at close range (hood near 0°) and far range (hood near saturation).

### Recommended Pass Distances (4 rows)

| Row | Distance (m) | Notes |
|-----|-------------|-------|
| 1   | 1.0         | Existing close point |
| 2   | 2.5         | New midpoint |
| 3   | 4.0         | New midpoint |
| 4   | 5.5         | Existing far point |

### Calibration Workflow (per distance)

1. **Mark distances on the floor** — use tape at each target distance from the hub.
2. **Position the robot** at the marked distance, facing the hub.
3. **Enable `testShoot`** — hold the right trigger in tuning mode.
4. **Read `TestShoot/distanceMeters`** from NetworkTables to get the exact turret-to-hub distance (this accounts for the turret offset from robot center).
5. **Adjust `Shooter/HoodAngleDeg`** on the dashboard until balls consistently enter the hub.
6. **Record** `{distance, hood angle, time-of-flight}` — measure TOF with a stopwatch or video frame count (launch → score).
7. **Move to the next distance** and repeat.
8. **Update `subsystems/shooter/ShooterLookupTables.java`** with the recorded values.

### Tips

- **Calibrate close → far** — start at 1.25 m and work outward. Close shots are easiest to verify.
- **Use `TestShoot/modelRPM`** to sanity-check the two-point flywheel model at each distance. If shots consistently fall short or overshoot, the model anchors (`ShooterModel.dMin/dMax/kRpmAtMin/kRpmAtMax`) may need updating.
- **Re-run the full sequence** after any mechanical change to the hood or flywheel (wheel swap, belt tension, etc.).
- **Time-of-flight** doesn't need to be precise to the millisecond — ±0.05 s is fine. It's used for lead compensation, not direct aiming.

---

## Contact
If you want further cleanup or clarification, say so and I'll prepare it.
