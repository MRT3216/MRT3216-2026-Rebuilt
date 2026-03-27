package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;

/** Shooter constants (flywheel, spindexer, kicker, hood, turret). */
public final class ShooterConstants {
    private ShooterConstants() {}

    // -------------------------------------------------------------------------
    // Shared / top-level shooter constants
    // -------------------------------------------------------------------------

    /**
     * Competition shoot modes, toggled by operator Y button.
     *
     * <ul>
     *   <li>{@link #FULL} — full SOTF: lead-compensated distance for RPM/hood, turret tracks azimuth
     *       (when azimuth tracking is enabled).
     *   <li>{@link #FULL_STATIC} — manual aim fallback: raw distance AND turret locked at 0°. The
     *       driver must face the hub manually.
     * </ul>
     */
    public enum ShootMode {
        /** Full shoot-on-the-fly with lead compensation. */
        FULL,
        /** Raw hub distance, turret locked at 0°. Manual aim fallback. */
        FULL_STATIC
    }

    /** Convergence threshold (meters) for shot refinement. */
    public static final Distance kRefinementConvergenceEpsilon = Meters.of(0.01);

    /**
     * Mid-match RPM fudge factor (absolute RPM offset). Added directly to the flywheel RPM computed
     * by the shooting model: {@code finalRPM = modelRPM + fudge}.
     *
     * <p>Adjusted in 50 RPM increments by the operator via RB (+50) / LB (−50), clamped to ±200 RPM.
     * Also accessible via the Elastic dashboard Number Slider. Positive values increase RPM (shots
     * landing short), negative values decrease RPM (shots overshooting). Always active regardless of
     * {@link ShootMode}. Published to NetworkTables unconditionally so the operator can see the
     * current value on the dashboard.
     */
    public static final LoggedTunableNumber kRPMFudgeRPM =
            new LoggedTunableNumber("Shooter/RPMFudge", 0.0, true);

    /**
     * Mid-match distance fudge factor (meters). Added to the lead distance before the shooting model
     * lookup: {@code effectiveDistance = leadDistance + fudge}.
     *
     * <p>Positive values pretend the hub is farther away (more RPM, steeper hood — use when shots
     * land short). Negative values pretend the hub is closer (less RPM, flatter hood — use when shots
     * overshoot). Always active regardless of {@link ShootMode}. Published to NetworkTables
     * unconditionally so the operator can adjust mid-match via the dashboard slider.
     */
    public static final LoggedTunableNumber kDistanceFudgeMeters =
            new LoggedTunableNumber("Shooter/DistanceFudgeMeters", 0.0, true);

    // -------------------------------------------------------------------------
    // Flywheel (velocity)
    // -------------------------------------------------------------------------

    public static final class FlywheelConstants {
        private FlywheelConstants() {}

        // Mechanical
        public static final Distance kWheelDiameter = Inches.of(3);
        public static final Mass kWheelMass = Pounds.of(3);
        public static final double kGearReduction = 1.0;

        // Motor wiring
        public static final Current kStatorCurrentLimit = Amps.of(80);

        // PID — Voltage mode.
        // Reference team values (TorqueFOC, not directly comparable):
        //   6328: kP=0.4, Hammerheads: kP=12, WHS 3467: kP=12
        // Our 0.2 is at the low end for Voltage mode. Increase to 0.5-1.0 if
        // the flywheel is slow to recover RPM after a shot.
        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward
        public static final double kS = 0.35;
        public static final double kV = 0.12;
        public static final double kA = 0.0;

        // Simulation overrides
        public static final double kP_sim = 0.1;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.124;
        public static final double kA_sim = 0.1;

        /** Returns a preconfigured feedforward for the flywheel. */
        public static SimpleMotorFeedforward motorFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the flywheel feedforward. */
        public static SimpleMotorFeedforward motorFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }

        // Soft limits
        public static final AngularVelocity kSoftLimitMax = RPM.of(5000.0);
        public static final AngularVelocity kSoftLimitMin = RPM.of(0.0);

        // Targets / tunables
        public static final AngularVelocity kFlywheelDefaultVelocity = RPM.of(3000);
        public static final AngularVelocity kVelocityTolerance = RPM.of(30);
        public static final Time kClearDuration = Seconds.of(0.25);

        public static final LoggedTunableNumber kTunableFlywheelRPM =
                new LoggedTunableNumber(
                        "Shooter/FlywheelRPM", kFlywheelDefaultVelocity.in(RPM), Constants.tuningMode);
    }

    // -------------------------------------------------------------------------
    // Shooter model (two-point linear RPM lookup)
    // -------------------------------------------------------------------------

    /**
     * Simple two-point shooter model constants used for a lightweight linear RPM model.
     *
     * <p>Values are unit-aware. Distances are stored as {@link Distance} and flywheel anchors are
     * stored as {@link AngularVelocity} using RPM units.
     */
    public static final class ShooterModel {
        private ShooterModel() {}

        public static final Distance dMin = Inches.of(61);
        public static final Distance dMax = Inches.of(291.5);

        public static final AngularVelocity kRpmAtMin = RPM.of(2500.0);
        public static final AngularVelocity kRpmAtMax = RPM.of(4300.0);
    }

    // -------------------------------------------------------------------------
    // Spindexer (velocity)
    // -------------------------------------------------------------------------

    public static final class SpindexerConstants {
        private SpindexerConstants() {}

        // Mechanical
        public static final Distance kWheelDiameter = Inches.of(8);
        public static final Mass kWheelMass = Pounds.of(2);
        public static final double kGearReduction = 5.0;

        // Motor wiring
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // PID
        // Startup spike to ~60 is consistent across all kP/kD values — it's
        // from the instantaneous FF step, not correctable by PID alone.
        // kP=0.05 + kD=0.005 keeps steady-state close and doesn't add to spike.
        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 0.005;

        // Feedforward
        // kV=0.5 settled ~4 RPM low. Bumped to 0.52 for tighter steady-state.
        public static final double kS = 0.25;
        public static final double kV = 0.52;
        public static final double kA = 0.0;

        // Simulation overrides
        public static final double kP_sim = 0.01;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.624;
        public static final double kA_sim = 0.0;

        /** Returns a preconfigured feedforward for the spindexer. */
        public static SimpleMotorFeedforward motorFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the spindexer feedforward. */
        public static SimpleMotorFeedforward motorFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }

        // Soft limits
        public static final AngularVelocity kSoftLimitMax = RPM.of(1000.0);
        public static final AngularVelocity kSoftLimitMin = RPM.of(0.0);

        // Targets / tunables
        public static final AngularVelocity kSpindexerTargetAngularVelocity = RPM.of(500.0);

        public static final LoggedTunableNumber kTunableIndexerRPM =
                new LoggedTunableNumber(
                        "Spindexer/IndexerRPM", kSpindexerTargetAngularVelocity.in(RPM), Constants.tuningMode);
    }

    // -------------------------------------------------------------------------
    // Kicker (velocity)
    // -------------------------------------------------------------------------

    public static final class KickerConstants {
        private KickerConstants() {}

        // Mechanical
        public static final Distance kWheelDiameter = Inches.of(2.5);
        public static final Mass kWheelMass = Pounds.of(1);
        public static final double kGearReduction = 1.0;

        // Motor wiring
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // PID — intentionally zero: kicker runs feedforward-only on the real robot.
        // Add PID if the kicker stalls on ball contact (6328 uses kP=3.0).
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.0;

        // Simulation overrides
        public static final double kP_sim = 0.001;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.1045;
        public static final double kA_sim = 0.0;

        /** Returns a preconfigured feedforward for the kicker. */
        public static SimpleMotorFeedforward motorFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the kicker feedforward. */
        public static SimpleMotorFeedforward motorFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }

        // Soft limits
        public static final AngularVelocity kSoftLimitMax = RPM.of(4000.0);
        public static final AngularVelocity kSoftLimitMin = RPM.of(0.0);

        // Targets / tunables
        public static final AngularVelocity kKickerTargetAngularVelocity = RPM.of(2500.0);
        public static final AngularVelocity kKickerClearAngularVelocity = RPM.of(-100.0);

        public static final LoggedTunableNumber kTunableKickerRPM =
                new LoggedTunableNumber(
                        "Kicker/KickerRPM", kKickerTargetAngularVelocity.in(RPM), Constants.tuningMode);
    }

    // -------------------------------------------------------------------------
    // Hood (positional)
    // -------------------------------------------------------------------------

    public static final class HoodConstants {
        private HoodConstants() {}

        // Mechanical
        public static final double kGearing = 30.0;
        public static final Distance kLength = Inches.of(0.5);
        public static final Mass kMass = Pounds.of(0.1);

        // Motor wiring
        public static final boolean kMotorInverted = true;
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // PID — Voltage mode (TalonFX Kraken X44, 30:1 gearing).
        // Reference team values (TorqueFOC, not directly comparable):
        //   6328: kP=1200/kD=4, Hammerheads: kP=800/kD=5, WHS 3467: kP=3000/kD=160
        // Our 300 is reasonable for Voltage mode. Increase kD if the hood oscillates
        // around setpoint; add kD cautiously since 30:1 gearing amplifies noise.
        public static final double kP = 300.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward
        public static final double kS = 0.45;
        public static final double kV = 3.0;
        public static final double kA = 0.0;

        // Simulation overrides
        public static final double kP_sim = 5.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.3;
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 3.7;
        public static final double kA_sim = 0.0;

        /** Returns a preconfigured feedforward for the hood pivot. */
        public static SimpleMotorFeedforward pivotFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the hood feedforward. */
        public static SimpleMotorFeedforward pivotFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }

        // Motion-profile limits (used by the closed-loop controller)
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(270.0);
        public static final AngularAcceleration kMaxAccel = DegreesPerSecondPerSecond.of(270.0);

        // Hard limits
        public static final Angle kHardLimitMax = Degrees.of(30);
        public static final Angle kHardLimitMin = Degrees.of(0);

        // Soft limits
        public static final Angle kSoftLimitMax = Degrees.of(30);
        public static final Angle kSoftLimitMin = Degrees.of(0);

        // Presets / tunables
        public static final Angle kStartingPosition = Degrees.of(0);
        public static final Angle kTolerance = Degrees.of(0.5);

        public static final LoggedTunableNumber kTunableHoodAngleDeg =
                new LoggedTunableNumber(
                        "Shooter/HoodAngleDeg", kStartingPosition.in(Degrees), Constants.tuningMode);

        // Duck behaviour
        public static final Distance EXTRA_DUCK_DISTANCE = Inches.of(12.0);
        public static final Time kDuckDuration = Seconds.of(0.5);
        public static final Angle kDuckAngle = Degrees.of(15.0);
    }

    // -------------------------------------------------------------------------
    // Turret (positional)
    // -------------------------------------------------------------------------

    public static final class TurretConstants {
        private TurretConstants() {}

        // Mechanical
        public static final double kGearing = 27.0;
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0502269403);

        // Motor wiring
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // PID — units are Volts per mechanism ROTATION of error (not degrees).
        // Because YAMS sets positionConversionFactor = 1/gearing, the SparkMax's
        // internal PID sees position in mechanism rotations.
        //
        // Plain position PID (no MAXMotion). kP for accuracy, kD for damping.
        public static final double kP = 3.0;
        public static final double kI = 0.0;
        public static final double kD = 0.3;

        // Feedforward
        public static final double kS = 0.0;
        public static final double kV = 1.0;
        public static final double kA = 0.05;

        // Motion profile — kept for sim config only (not used on real robot).
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(720.0);
        public static final AngularAcceleration kMaxAccel = DegreesPerSecondPerSecond.of(3600.0);

        // Simulation overrides
        public static final double kP_sim = 3.5;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 2.0;
        public static final double kA_sim = 0.03;

        // Robot-to-turret transform
        public static final Distance kTurretOffsetX = Inches.of(-5.5);
        public static final Distance kTurretOffsetY = Inches.of(5.5);
        public static final Distance kTurretOffsetZ = Inches.of(13.0625);
        public static final Transform3d kRobotToTurretTransform =
                new Transform3d(
                        new Translation3d(
                                kTurretOffsetX.in(Meters), kTurretOffsetY.in(Meters), kTurretOffsetZ.in(Meters)),
                        new Rotation3d());

        // Hard limits (physical stops — sim only)
        public static final Angle kHardLimitMax = Degrees.of(135);
        public static final Angle kHardLimitMin = Degrees.of(-95);

        // Soft limits (closed-loop clamp)
        public static final Angle kSoftLimitMax = Degrees.of(130);
        public static final Angle kSoftLimitMin = Degrees.of(-90);

        // Presets / tunables
        public static final Angle kStartingPosition = Degrees.of(0);

        // ── EasyCRT absolute-position bootstrapping ──
        // Set to true to attempt CRT solve at boot; false falls back to kStartingPosition.
        public static final boolean kUseCRT = true;

        // Encoder gearing: both absolute encoders mesh with the 90T ring gear (= turret).
        // commonRatio = 1.0 because the ring gear IS the turret (1:1).
        // Encoder 1 = REV Through Bore on SparkMax absolute-encoder port (10T pinion).
        // Encoder 2 = PWM absolute encoder on RoboRIO DIO (13T pinion).
        public static final double kCRTCommonRatio = 1.0;
        public static final int kCRTDriveGearTeeth = 90;
        public static final int kCRTEncoder1PinionTeeth = 10; // SparkMax abs enc — 90/10 = 9:1
        public static final int kCRTEncoder2PinionTeeth = 13; // RoboRIO PWM enc  — 90/13 ≈ 6.923:1

        // Mechanism range — derived from the hard limits with a small margin so the CRT
        // solver's coverage window always exceeds the physical travel.  Changing the hard
        // limits automatically updates this; no second constant to keep in sync.
        private static final double kCRTRangeMarginRot = 0.05; // ≈ 18° extra on each side
        public static final Angle kCRTMechanismMin =
                Rotations.of(kHardLimitMin.in(Rotations) - kCRTRangeMarginRot);
        public static final Angle kCRTMechanismMax =
                Rotations.of(kHardLimitMax.in(Rotations) + kCRTRangeMarginRot);

        // Encoder offsets (rotations, added before wrap).
        // Calibrated 2025-03-24: averaged 4 boot samples at mechanical zero (sharpie mark).
        //   Enc1 raw avg: 0.821,  Enc2 raw avg: 0.805
        // Previous values: -0.7983 / -0.86415 (enc2 was 0.06 rot off → ~0.09 ErrorRot).
        public static final Angle kCRTEncoder1Offset = Rotations.of(-0.821);
        public static final Angle kCRTEncoder2Offset = Rotations.of(-0.805);

        // Match tolerance: allowable modular error between predicted and measured encoder 2.
        // After recalibrating offsets and fixing swapped pinion teeth (2025-03-24):
        // After fixing swapped pinion teeth and recalibrating offsets (2025-03-24),
        // worst-case ErrorRot across 7 positions (0° to ±180°) was 0.008.
        // Set to 0.02 — 2.5× margin over worst case, well below
        // the 0.144 rot candidate spacing (no AMBIGUOUS risk).
        public static final Angle kCRTMatchTolerance = Rotations.of(0.02);

        // Encoder inversions (set true if an encoder reads backwards w.r.t. mechanism positive).
        public static final boolean kCRTEncoder1Inverted = false;
        public static final boolean kCRTEncoder2Inverted = false;
    }

    // -------------------------------------------------------------------------
    // Hybrid Aiming (drivetrain coarse + turret fine)
    // -------------------------------------------------------------------------

    /**
     * Constants for hybrid aiming mode where the <em>drivetrain</em> handles coarse heading
     * correction and the <em>turret</em> handles only the residual fine adjustment.
     *
     * <p><b>Why hybrid?</b> A full-rotation turret stresses the wiring loom every time it swings
     * across large angles. In hybrid mode the turret stays within an asymmetric comfort zone ({@link
     * #kTurretMinDeg}° to {@link #kTurretMaxDeg}°), dramatically reducing cable wear. The drivetrain
     * rotates the whole robot to keep the target roughly in front, and the turret corrects the last
     * few degrees.
     *
     * <p><b>How it works:</b>
     *
     * <ol>
     *   <li>Each loop, compute the <em>full</em> robot-relative angle to the target (same math as the
     *       existing {@code HybridTurretUtil}).
     *   <li>If that angle is within the turret travel window ({@code kTurretMinDeg} to {@code
     *       kTurretMaxDeg}), the drivetrain does nothing and the turret tracks normally — the driver
     *       retains full rotational control.
     *   <li>If the angle exceeds the deadband, the drivetrain heading controller kicks in and rotates
     *       the robot toward the target, bleeding off error until the turret can handle the
     *       remainder.
     *   <li>The turret always receives the <em>actual</em> residual error (full angle minus whatever
     *       the drivetrain has already corrected), so it never needs to travel far.
     * </ol>
     *
     * <p><b>Currently active.</b> Wired in {@code RobotContainer} for both competition and tuning
     * modes. The relevant commands are {@code ShooterSystem.hybridAimAndShoot()} (and variants) and
     * {@code DriveCommands.joystickDriveAimAtTarget()}.
     *
     * <p>To revert to turret-only aiming, replace {@code hybridAimAndShoot} with {@code aimAndShoot}
     * and swap the drive default back to {@code DriveCommands.joystickDrive()}.
     */
    public static final class HybridAimingConstants {
        private HybridAimingConstants() {}

        /**
         * Turret "home" angle in degrees — the direction the shooter faces when the turret is at its
         * resting position, measured relative to the robot's front.
         *
         * <ul>
         *   <li>{@code 0.0} — shooter faces forward (default).
         *   <li>{@code 180.0} — shooter faces backward. The drivetrain will aim the <em>rear</em> of
         *       the robot at the target, and the turret clamp will center around 180° instead of 0°.
         *   <li>Any other value works too (e.g. 90° for a side-mounted shooter).
         * </ul>
         *
         * <p>This offset is applied in two places:
         *
         * <ol>
         *   <li><b>DriveCommands.joystickDriveAimAtTarget</b> — the drivetrain heading setpoint is
         *       offset so the correct face of the robot points at the target.
         *   <li><b>ShooterSystem.hybridAimAndShoot</b> — the turret clamp is centered around this angle
         *       instead of 0°.
         * </ol>
         */
        public static final double kTurretHomeAngleDeg = 0.0;

        /**
         * Turret travel limits in degrees, relative to the home angle. These define the asymmetric
         * window the turret is allowed to sweep before the drivetrain heading controller takes over.
         *
         * <p>{@code kTurretMinDeg} is the most-negative (clockwise) limit and {@code kTurretMaxDeg} is
         * the most-positive (counter-clockwise) limit. For example, {@code -90} / {@code +130} gives
         * 90° of clockwise travel and 130° of counter-clockwise travel from the home angle.
         *
         * <p>The drivetrain ramp zone (see {@link #kThresholdMarginDeg}) applies independently on each
         * side — the ramp starts at {@code limit ± margin} and reaches full correction at the limit.
         */
        public static final double kTurretMinDeg = -90.0;

        public static final double kTurretMaxDeg = 130.0;

        /**
         * Margin (in degrees) before the turret reaches each travel limit where the drivetrain begins
         * ramping in heading correction. Instead of a hard on/off at the limit boundary, the drivetrain
         * assist fades in linearly over this margin — giving it a head start so the chassis is already
         * rotating by the time the turret reaches its clamp limit.
         *
         * <p><b>Zones (measured from turret home, per side):</b>
         *
         * <ul>
         *   <li><b>0° – (limit − margin)°:</b> Turret-only zone. Drivetrain does nothing.
         *   <li><b>(limit − margin)° – limit°:</b> Ramp zone. Drivetrain correction scales linearly
         *       from 0% → 100%.
         *   <li><b>&gt; limit°:</b> Full correction — same as before.
         * </ul>
         *
         * <p>With min=-90°, max=+130° and margin=15°: the drivetrain starts helping at -75° / +115° and
         * reaches full strength at -90° / +130°. Inspired by 254/1323/4481's 2022 graduated
         * turret-wrap-prevention systems.
         *
         * <p>Set to 0.0 to disable the ramp and revert to hard on/off behavior.
         */
        public static final double kThresholdMarginDeg = 15.0;

        /**
         * PID gains for the drivetrain heading controller used in hybrid aiming mode. These control how
         * aggressively the robot chassis rotates toward the target when the turret angle exceeds its
         * travel limits.
         *
         * <p>Start conservative — the driver should barely notice the heading correction. Increase kP
         * if the robot is too sluggish snapping to the target; add kD if it overshoots.
         *
         * <p>Units: radians/sec output per radian of heading error (continuous input ±π).
         */
        public static final double kHeadingKP = 3.0;

        public static final double kHeadingKI = 0.0;
        public static final double kHeadingKD = 0.2;

        /**
         * Maximum rotational velocity (rad/s) and acceleration (rad/s²) for the profiled heading
         * controller. Limits how fast the drivetrain can snap to the target heading.
         *
         * <p>Lower values make the correction smoother and less jarring for the driver; higher values
         * get on-target faster but may feel aggressive.
         */
        public static final double kHeadingMaxVelocity = 4.0; // rad/s

        public static final double kHeadingMaxAcceleration = 8.0; // rad/s²
    }
}
