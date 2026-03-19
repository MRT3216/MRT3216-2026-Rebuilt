// Shooter constants (flywheel, spindexer, kicker, hood, turret). Tuned by the team.
package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
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
import frc.robot.util.LoggedTunableNumber;

/** Flywheel shooter constants: wheel geometry, control gains, and velocities. */
public final class ShooterConstants {
    private ShooterConstants() {}

    /** Convergence threshold (meters) for shot refinement. */
    public static final Distance kRefinementConvergenceEpsilon = Meters.of(0.01); // 1 cm

    // Group flywheel-specific constants under a Flywheel section to improve
    // organization.
    public static final class FlywheelConstants {
        private FlywheelConstants() {}

        // Mechanical
        public static final Distance kWheelDiameter = Inches.of(3);
        public static final Mass kWheelMass = Pounds.of(3);
        public static final double kGearReduction = 1.0;

        // Electrical limits
        public static final Current kStatorCurrentLimit = Amps.of(80);

        // PID gains (velocity)
        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward gains
        public static final double kS = 0.35;
        public static final double kV = 0.12;
        public static final double kA = 0.0;

        // Simulation overrides
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.124;
        public static final double kA_sim = 0.1;
        public static final double kP_sim = 0.1;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;

        // Telemetry keys are centralized in TelemetryKeys

        // Recommended target velocities
        public static final AngularVelocity kFlywheelDefaultVelocity = RPM.of(3000);
        public static final AngularVelocity kVelocityTolerance = RPM.of(30);

        /** Soft limits (RPM) used for safety and tooling. */
        public static final AngularVelocity kSoftLimitMax = RPM.of(5000.0);

        public static final AngularVelocity kSoftLimitMin = RPM.of(0.0);

        /** Clear routine duration (seconds). */
        public static final double kClearDurationSecs = 0.25;

        /**
         * Returns a preconfigured SimpleMotorFeedforward suitable for flywheel/feedforward use in
         * controllers. Keeps constructor semantics centralized to avoid copy/paste errors.
         */
        public static SimpleMotorFeedforward motorFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the motor feedforward. */
        public static SimpleMotorFeedforward motorFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }

        /** Tunable: flywheel target RPM. */
        public static final LoggedTunableNumber kTunableFlywheelRPM =
                new LoggedTunableNumber(
                        "Shooter/FlywheelRPM", kFlywheelDefaultVelocity.in(RPM), Constants.tuningMode);
    }

    /**
     * Simple two-point shooter model constants used for a lightweight linear RPM model.
     *
     * <p>Values are unit-aware. Distances are stored as {@link Distance} and flywheel anchors are
     * stored as {@link AngularVelocity} using RPM units so they are friendly for tuning with RPM
     * values.
     */
    public static final class ShooterModel {
        private ShooterModel() {}

        // Distance bounds (inches) for the shooter model.
        public static final Distance dMin = Inches.of(61);
        public static final Distance dMax = Inches.of(291.5);

        // Flywheel anchor speeds (RPM)
        public static final AngularVelocity kRpmAtMin = RPM.of(2500.0);
        public static final AngularVelocity kRpmAtMax = RPM.of(4300.0);
    }

    public static final class SpindexerConstants {
        private SpindexerConstants() {}

        // Mechanical
        public static final double kGearReduction = 5.0;

        /** Roller diameter. */
        public static final Distance kWheelDiameter = Inches.of(8);

        /** Roller mass. */
        public static final Mass kWheelMass = Pounds.of(2);

        // Motor wiring and limits
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // PID and feedforward
        public static final double kP = 0.02;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.25;
        public static final double kV = 0.6;
        public static final double kA = 0.0;

        // Simulation-specific defaults
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.624;
        public static final double kA_sim = 0.0;
        public static final double kP_sim = 0.01;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;

        // Recommended velocities
        // 700 RPM mechanism × 5.0 gear reduction = ~3500 RPM at the NEO shaft.
        public static final AngularVelocity kSpindexerTargetAngularVelocity = RPM.of(500.0);

        /** Soft limit (RPM). */
        public static final AngularVelocity kSoftLimitMax = RPM.of(1000.0);

        public static final AngularVelocity kSoftLimitMin = RPM.of(0.0);

        /** Returns a SimpleMotorFeedforward for the spindexer. */
        public static SimpleMotorFeedforward motorFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the spindexer feedforward. */
        public static SimpleMotorFeedforward motorFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }

        /** Tunable: spindexer target RPM. */
        public static final LoggedTunableNumber kTunableIndexerRPM =
                new LoggedTunableNumber(
                        "Spindexer/IndexerRPM", kSpindexerTargetAngularVelocity.in(RPM), Constants.tuningMode);
    }

    public static final class KickerConstants {
        private KickerConstants() {}

        // Mechanical
        public static final Distance kWheelDiameter = Inches.of(2.5);
        public static final Mass kWheelMass = Pounds.of(1);
        public static final double kGearReduction = 1.0;

        // Electrical limits
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // PID
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.0;

        // Simulation variants
        public static final double kS_sim = 0;
        public static final double kV_sim = 0.1045;
        public static final double kA_sim = 0;
        public static final double kP_sim = 0.001;
        public static final double kI_sim = 0;
        public static final double kD_sim = 0;

        // Recommended velocities
        public static final AngularVelocity kKickerTargetAngularVelocity = RPM.of(2500.0);
        public static final AngularVelocity kKickerClearAngularVelocity = RPM.of(-100.0);

        /** Soft limit (RPM). */
        public static final AngularVelocity kSoftLimitMax = RPM.of(4000.0);

        public static final AngularVelocity kSoftLimitMin = RPM.of(0.0);

        /** Returns a preconfigured SimpleMotorFeedforward for the kicker. */
        public static SimpleMotorFeedforward motorFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the kicker feedforward. */
        public static SimpleMotorFeedforward motorFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }

        /** Tunable: kicker target RPM. */
        public static final LoggedTunableNumber kTunableKickerRPM =
                new LoggedTunableNumber(
                        "Kicker/KickerRPM", kKickerTargetAngularVelocity.in(RPM), Constants.tuningMode);
    }

    public static final class HoodConstants {
        private HoodConstants() {}

        // Mechanical
        public static final double kGearing = 30.0;
        public static final Distance kLength = Inches.of(0.5);
        public static final Mass kMass = Pounds.of(0.1);

        // Motor wiring
        public static final boolean kMotorInverted = true;
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // PID and feedforward
        public static final double kP = 300.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.45;
        public static final double kV = 3.0;
        public static final double kA = 0.0;

        // Simulation-specific defaults for Hood
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 3.7;
        public static final double kA_sim = 0.0;
        public static final double kP_sim = 5.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.3;

        // Limits / presets
        public static final Angle kHardLimitMax = Degrees.of(30);
        public static final Angle kHardLimitMin = Degrees.of(0);
        public static final Angle kSoftLimitMax = Degrees.of(30);
        public static final Angle kSoftLimitMin = Degrees.of(0);

        // public static final Angle kHoodHorizontalOffset = Degrees.of(16.574);

        public static final Angle kStartingPosition = Degrees.of(0);

        /** Position tolerance for runTo (degrees). */
        public static final Angle kTolerance = Degrees.of(0.5);
        /** Tunable: hood angle in degrees. */
        public static final LoggedTunableNumber kTunableHoodAngleDeg =
                new LoggedTunableNumber(
                        "Shooter/HoodAngleDeg", kStartingPosition.in(Degrees), Constants.tuningMode);

        // Manual distance tunable removed — use spreadsheet/model constants instead.

        /** Returns a SimpleMotorFeedforward for the hood pivot. */
        public static SimpleMotorFeedforward pivotFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the hood feedforward. */
        public static SimpleMotorFeedforward pivotFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }

        public static final Distance EXTRA_DUCK_DISTANCE = Inches.of(12.0); // inches
        // TODO: tune these on robot: duck duration and duck angle.
        public static final Time kDuckDuration = Seconds.of(0.5);
        public static final Angle kDuckAngle = Degrees.of(15.0);
    }

    public static final class TurretConstants {
        private TurretConstants() {}

        // Mechanical
        public static final double kGearing = 27;

        // Motor wiring
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // Inertia / dynamics
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0502269403);

        // PIDs
        public static final double kP = 3.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 1.0;
        public static final double kA = 0.05;
        public static final AngularVelocity kMaxVelocity =
                DegreesPerSecond.of(1440.0); // degrees per second
        public static final AngularAcceleration kMaxAccel =
                DegreesPerSecondPerSecond.of(14400.0); // degrees per second
        // squared

        // Simulation-tuned PID defaults (reduced to avoid oscillation in sim)
        public static final double kP_sim = 3.5;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 2.0;
        public static final double kA_sim = 0.03;

        // Robot / turret pose offsets
        public static final Distance kTurretOffsetX = Inches.of(-5.5);
        public static final Distance kTurretOffsetY = Inches.of(5.5);
        public static final Distance kTurretOffsetZ = Inches.of(13.0625);
        public static final Transform3d kRobotToTurretTransform =
                new Transform3d(
                        new Translation3d(
                                kTurretOffsetX.in(Meters), kTurretOffsetY.in(Meters), kTurretOffsetZ.in(Meters)),
                        new Rotation3d());

        public static final Angle kTurretAngleOffset =
                Degrees.of(0); // degrees, added to turret angle to get actual
        // heading

        // Limits / presets
        public static final Angle kHardLimitMax = Degrees.of(200);
        public static final Angle kHardLimitMin = Degrees.of(-200);
        public static final Angle kSoftLimitMax = Degrees.of(195);
        public static final Angle kSoftLimitMin = Degrees.of(-195);
        public static final Angle kStartingPosition = Degrees.of(0);
    }
}
