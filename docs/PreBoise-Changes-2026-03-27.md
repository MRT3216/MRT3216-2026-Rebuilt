# Pre-Boise Code Changes — Week of March 20–27, 2026

---

## Before Practice Round (March 20 – March 26 ~7pm)

### Controls
- **Swapped driver/operator controls** — driver owns intake, operator owns shooting
- **Operator buttons remapped:** Y = shoot mode toggle (SOTF/static/full static), RB/LB = RPM fudge ±50
- **Driver LT = pass shot**, driver RT = hub shot
- **Single-controller tuning mode** with aim-only triggers and test shoot on X
- **D-pad snap angles** for turret in tuning mode

### Hybrid Aiming System (NEW)
- **Drivetrain + turret hybrid aiming:** drivetrain handles coarse heading toward target, turret handles the residual angle
- **Graduated drivetrain heading assist** — scales assistance based on how far off-target the turret is
- **Asymmetric turret clamp limits:** [−90°, +130°] travel window in hybrid mode
- **Turret default stow at home** instead of always tracking when not shooting
- **CRT (cross-referenced tracking) fix:** swapped pinion teeth, recalibrated offsets, added retry logic
- **Turret PID re-tuned:** kP=100, kV=3.4 (corrected for mechanism rotation units, not degrees)
- **Turret wrap-around:** software handles continuous rotation past ±180° so the turret never has to reverse direction to reach a target on the other side

### Shooter
- **testShoot reworked:** auto-RPM from distance using ShooterModel, tunable dashboard override, distance logged to NT
- **Shoot mode toggles + RPM fudge factor:** operator can adjust ±50 RPM mid-match via RB/LB (clamped to ±200)
- **Flywheel pre-spin** moved into competition branch only (stopHold in tuning mode)

### Autos
- **SOTF autos switched to HybridAimAndShoot** — autos now use the hybrid aiming system
- **SOTF auto adjustments** — timing and path tweaks
- **Abram's auto added**

### Dashboard (Elastic)
- **Teleop tab redesigned** like 6328's co-pilot shift dashboard with status widgets
- **Field2d added** to Disabled tab
- **Shooting LEDs wired** — visual feedback for locked-on, feeding, etc.

### Subsystem Cleanup
- **LED colors updated** to team teal/orange, removed climbing patterns
- **LEDSubsystem reworked:** standard constructor (no singleton), alternating locked-on pattern, Mechanism2D sim
- **Constants reorganized** into subsystem packages
- **All 8 subsystems standardized** in code organization
- **Dead code removed** across RobotContainer, ShooterSystem, IntakeSystem

### Infrastructure
- **Phoenix bumped to 26.1.3**
- **LoggedTunableNumber synced** with latest
- **Telemetry optimized** — reduced verbosity, conditional logging
- **BatteryLogger added**
- **Robot mass updated** to 63.5 kg (140 lbs)
- **PathPlanner settings.json fixed** to match actual robot dimensions
- **Defense mode added**

### Documentation
- **TuningGuide rewritten** for high school students with tables and analogies
- **ControllerGuide rewritten** for new button layout
- **Tuning checklist** extracted as standalone doc
- **Reference team gain comparisons** added to constants and docs

---

## After Practice Round (March 26 ~8pm – March 27)

### 1. Telemetry & Performance Optimization
Reduced unnecessary telemetry logging, optimized LED update rates, turret and intake periodic overhead. Addresses loop overrun issues seen in practice.

### 2. Kicker Clearing Fix
The kicker reverse-clear before feeding wasn't actually clearing jammed balls. **Fixed:** increased clear duration from 0.25s → 0.5s and reverse speed from −100 → −500 RPM (10× more reverse rotation). Balls should no longer get stuck in the kicker.

### 3. Feed Timing Fix (Shift-Gated Shooting)
Previously the kicker clear ran *inside* the shift window, eating into scoring time. **Fixed:** the kicker now clears immediately when RT is pulled, and the actual feed waits for the shift window to open. We start feeding the instant the shift goes active instead of wasting ~0.5s clearing first.

### 4. Drive Speed Reduction While Feeding
**New behavior:** When actively feeding balls into the hub (RT held AND shift is active), drive speed drops to **30%**. This only applies during the actual feed — full speed during spinup/aiming. The heading-assist PID is unaffected so aim tracking stays sharp. Should significantly improve shot accuracy.

### 5. Pass Shot System Overhaul
The pass shot feature had placeholder values and was never functional. **Now working:**
- **New pass RPM model:** 4000–4800 RPM (higher than hub's 2500–4300) so the ball arcs high over the hub
- **New PASS lookup table:** 5 entries covering 3.5m–10m, steep hood angles (0–4°) for high-arc lob trajectories
- **RPM is separate from hub shots** — pass commands use their own RPM curve
- **All pass values are starting estimates** — will need field tuning

### What's Unchanged After Practice
- Hub shooting is completely untouched — same RPM model, same LUT, same behavior
- Controller bindings are the same
- Auto routines are the same
