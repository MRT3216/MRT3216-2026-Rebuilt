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
     * Competition shoot modes, toggled by operator stick presses.
     *
     * <ul>
     *   <li>{@link #FULL} — full SOTF: lead-compensated distance for RPM/hood, turret tracks azimuth
     *       (when azimuth tracking is enabled).
     *   <li>{@link #STATIC_DISTANCE} — SOTF disabled: uses raw turret-to-hub distance for RPM/hood,
     *       but turret still tracks azimuth (future-proofing for when azimuth is enabled).
     *   <li>{@link #FULL_STATIC} — proven comp mode: raw distance AND turret locked at 0°.
     * </ul>
     */
    public enum ShootMode {
        /** Full shoot-on-the-fly with lead compensation. */
        FULL,
        /** Raw hub distance (no lead), turret still tracks azimuth. */
        STATIC_DISTANCE,
        /** Raw hub distance, turret locked at 0°. Battle-tested comp fallback. */
        FULL_STATIC
    }

    /** Convergence threshold (meters) for shot refinement. */
    public static final Distance kRefinementConvergenceEpsilon = Meters.of(0.01);

    /**
     * Mid-match RPM fudge factor (percentage). Applied as a multiplier to the flywheel RPM computed
     * by the two-point model: {@code finalRPM = modelRPM × (1 + fudge/100)}.
     *
     * <p>Positive values increase RPM (shots landing short), negative values decrease RPM (shots
     * overshooting). Always active regardless of {@link ShootMode}. Published to NetworkTables
     * unconditionally so the operator can adjust mid-match from the dashboard.
     */
    public static final LoggedTunableNumber kRPMFudgePercent =
            new LoggedTunableNumber("Shooter/RPMFudgePercent", 0.0, true);

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
        public static final double kP = 0.02;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward
        public static final double kS = 0.25;
        public static final double kV = 0.6;
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
        // FF-only kicker is common: BroncBotz 3481 also uses kP=0 with kS/kV only.
        // 6328 uses kP=3.0 on their kicker — add PID if ours stalls on ball contact.
        // TODO: Re-tune PID for all mechanisms during Monday bring-up.
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

        // PID
        public static final double kP = 3.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward
        public static final double kS = 0.0;
        public static final double kV = 1.0;
        public static final double kA = 0.05;

        // Motion profile
        // NEO free speed = 5676 RPM through 27:1 = ~1261°/s max mechanism speed.
        // Set cruise velocity to ~80% of theoretical max for headroom.
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(1000.0);
        public static final AngularAcceleration kMaxAccel = DegreesPerSecondPerSecond.of(7200.0);

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
        public static final Angle kHardLimitMax = Degrees.of(195);
        public static final Angle kHardLimitMin = Degrees.of(-195);

        // Soft limits (closed-loop clamp)
        public static final Angle kSoftLimitMax = Degrees.of(190);
        public static final Angle kSoftLimitMin = Degrees.of(-190);

        // Presets / tunables
        public static final Angle kStartingPosition = Degrees.of(0);

        // ── EasyCRT absolute-position bootstrapping ──
        // Set to true to attempt CRT solve at boot; false falls back to kStartingPosition.
        public static final boolean kUseCRT = false;

        // Encoder gearing: both absolute encoders mesh with the 90T ring gear (= turret).
        // commonRatio = 1.0 because the ring gear IS the turret (1:1).
        // Encoder 1 = REV Through Bore on SparkMax absolute-encoder port (13T pinion).
        // Encoder 2 = PWM absolute encoder on RoboRIO DIO (10T pinion).
        public static final double kCRTCommonRatio = 1.0;
        public static final int kCRTDriveGearTeeth = 90;
        public static final int kCRTEncoder1PinionTeeth = 13; // SparkMax abs enc — 90/13 ≈ 6.923:1
        public static final int kCRTEncoder2PinionTeeth = 10; // RoboRIO PWM enc  — 90/10 = 9:1

        // Mechanism range — derived from the hard limits with a small margin so the CRT
        // solver's coverage window always exceeds the physical travel.  Changing the hard
        // limits automatically updates this; no second constant to keep in sync.
        private static final double kCRTRangeMarginRot = 0.05; // ≈ 18° extra on each side
        public static final Angle kCRTMechanismMin =
                Rotations.of(kHardLimitMin.in(Rotations) - kCRTRangeMarginRot);
        public static final Angle kCRTMechanismMax =
                Rotations.of(kHardLimitMax.in(Rotations) + kCRTRangeMarginRot);

        // Encoder offsets (rotations, added before wrap).
        // Calibrate: at mechanical zero, log raw encoder readings, then set these
        // so both read ≈ 0.0 at the zero pose.
        public static final Angle kCRTEncoder1Offset = Rotations.of(0.0);
        public static final Angle kCRTEncoder2Offset = Rotations.of(0.0);

        // Match tolerance: allowable modular error between predicted and measured encoder 2.
        // 0.02 rot ≈ 7.2° at the mechanism for our gearing — generous for initial bring-up.
        // Tighten after verifying backlash/noise characteristics.
        public static final Angle kCRTMatchTolerance = Rotations.of(0.02);

        // Encoder inversions (set true if an encoder reads backwards w.r.t. mechanism positive).
        public static final boolean kCRTEncoder1Inverted = false;
        public static final boolean kCRTEncoder2Inverted = false;
    }
}
