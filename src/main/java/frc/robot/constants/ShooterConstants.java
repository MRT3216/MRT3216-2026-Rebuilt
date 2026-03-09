// Shooter constants extracted from `Constants.java` for easier maintenance and smaller file churn.
// @source: measured and tuned by Littleton Robotics team (2025-2026)
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

/** Flywheel shooter constants: wheel geometry, control gains, and velocities. */
public final class ShooterConstants {
    private ShooterConstants() {}

    /**
     * Convergence threshold (meters) used by shot-refinement logic. Placed at the ShooterConstants
     * level because the refinement is a system-level concern that may be referenced from multiple
     * shooter-related classes.
     */
    public static final Distance kRefinementConvergenceEpsilon = Meters.of(0.01); // 1 cm

    // Group flywheel-specific constants under a Flywheel section to improve
    // organization.
    public static final class FlywheelConstants {
        private FlywheelConstants() {}

        // Mechanical
        public static final Distance kWheelDiameter = Inches.of(3);
        public static final Mass kWheelMass = Pounds.of(3);
        public static final double kGearReduction = 1.0;

        // Electrical / limits
        public static final Current kStatorCurrentLimit = Amps.of(80);

        // PID (velocity) - set to 0.0 for initial tuning
        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward - set to 0.0 for initial tuning
        public static final double kS = 0.35;
        public static final double kV = 0.12;
        public static final double kA = 0.0;

        // Simulation overrides - start at 0.0 until tuned
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.124;
        public static final double kA_sim = 0.1;
        public static final double kP_sim = 0.1;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;

        // Telemetry keys are centralized in TelemetryKeys

        // Recommended target velocities / helper constants
        public static final AngularVelocity kFlywheelPrepAngularVelocity = RPM.of(3000);

        /**
         * Soft limits (human units) used for SysId / safety tooling. Defaults are conservative; tune on
         * robot.
         */
        public static final AngularVelocity kSoftLimitMax = RPM.of(5000.0);

        public static final AngularVelocity kSoftLimitMin = RPM.of(0.0);

        /** Duration to run the clear routine while the flywheel spins up (seconds). */
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
    }

    /**
     * Simple two-point shooter model constants used for a lightweight linear RPM model.
     *
     * <p>Values are unit-aware. Distances are stored as {@link Distance} and flywheel anchors are
     * stored as {@link edu.wpi.first.units.measure.AngularVelocity} using RPM units so they are
     * friendly for tuning with RPM values.
     */
    public static final class ShooterModel {
        private ShooterModel() {}

        // Distance bounds for the linear model (meters)
        public static final Distance dMin = Meters.of(1.0);
        public static final Distance dMax = Meters.of(6.0);

        // Flywheel anchor speeds expressed in RPM for easy tuning on-robot.
        // Updated per user request: close = 2500 RPM, far = 3500 RPM.
        public static final edu.wpi.first.units.measure.AngularVelocity kRpmAtMin = RPM.of(2500.0);
        public static final edu.wpi.first.units.measure.AngularVelocity kRpmAtMax = RPM.of(3500.0);
    }

    public static final class SpindexerConstants {
        private SpindexerConstants() {}

        // Mechanical
        public static final double kGearReduction = 5.0;

        /** Roller physical size used by FlyWheel configs. */
        public static final Distance kWheelDiameter = Inches.of(4);

        /** Roller mass used by FlyWheel configs. */
        public static final Mass kWheelMass = Pounds.of(0.5);

        // Motor wiring / limits
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(60);

        // PID / Feedforward
        public static final double kP = 0.02;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.25;
        public static final double kV = 0.6;
        public static final double kA = 0.0;

        // Simulation-specific feedforward / PID defaults for Spindexer
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.624;
        public static final double kA_sim = 0.0;
        public static final double kP_sim = 0.01;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;

        // Telemetry keys are centralized in TelemetryKeys

        // Recommended / helper velocities
        public static final AngularVelocity kSpindexerTargetAngularVelocity = RPM.of(1200.0);

        /**
         * Soft limits (human units) used for SysId / safety tooling. Defaults are conservative; tune on
         * robot.
         */
        public static final AngularVelocity kSoftLimitMax = RPM.of(1500.0);

        public static final AngularVelocity kSoftLimitMin = RPM.of(-500.0);

        /** Returns a preconfigured SimpleMotorFeedforward for the spindexer. */
        public static SimpleMotorFeedforward motorFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the spindexer feedforward. */
        public static SimpleMotorFeedforward motorFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }
    }

    public static final class KickerConstants {
        private KickerConstants() {}

        // Mechanical
        public static final Distance kWheelDiameter = Inches.of(2.5);
        public static final Mass kWheelMass = Pounds.of(1);
        public static final double kGearReduction = 1.0;

        // Electrical / limits
        public static final Current kStatorCurrentLimit = Amps.of(60);

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

        // Telemetry keys are centralized in TelemetryKeys

        // Recommended velocities
        public static final AngularVelocity kKickerTargetAngularVelocity = RPM.of(2500.0);
        public static final AngularVelocity kKickerClearAngularVelocity = RPM.of(-100.0);

        /**
         * Soft limits (human units) used for SysId / safety tooling. Defaults are conservative; tune on
         * robot.
         */
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
    }

    public static final class HoodConstants {
        private HoodConstants() {}

        // Mechanical
        /** Gearing from motor to hood pivot (ratio). */
        public static final double kGearing = 30.0;

        /** Physical arm length from pivot to hood center (useful for dynamics) */
        public static final Distance kLength = Inches.of(0.5);

        /** Approximate mass of the hood assembly. */
        public static final Mass kMass = Pounds.of(0.1);

        // Motor wiring
        public static final boolean kMotorInverted = true;
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // PID & Feedforward
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        // Simulation-specific feedforward / PID defaults for Hood (tuned for sim)
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 3.7;
        public static final double kA_sim = 0.0;
        public static final double kP_sim = 5.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.3;

        // Motion limits
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(90.0);
        // Acceleration used for trapezoidal profiling (deg/s^2)
        public static final AngularAcceleration kMaxAccel = DegreesPerSecondPerSecond.of(90.0);

        // Limits / presets
        public static final Angle kHardLimitMax = Degrees.of(30);
        public static final Angle kHardLimitMin = Degrees.of(0);
        public static final Angle kSoftLimitMax = Degrees.of(30);
        public static final Angle kSoftLimitMin = Degrees.of(0);
        public static final Angle kStartingPosition = Degrees.of(0);

        // Telemetry keys are centralized in TelemetryKeys
        /** Allowed absolute position error for hood angle comparisons (degrees). */

        /**
         * Returns a preconfigured SimpleMotorFeedforward for the Hood pivot.
         *
         * <p>Notes: - Units: velocity is in rad/s for WPILib SimpleMotorFeedforward. - This is
         * intentionally a SimpleMotorFeedforward (ks, kv, ka). If the hood becomes heavier or its
         * gravity term grows, consider switching to ArmFeedforward to include a gravity term.
         */
        public static SimpleMotorFeedforward pivotFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the hood feedforward. */
        public static SimpleMotorFeedforward pivotFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }

        public static final Distance EXTRA_DUCK_DISTANCE = Inches.of(12.0); // inches
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

        // PID
        public static final double kP = 6.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Simulation-tuned PID defaults (reduced to avoid oscillation in sim)
        public static final double kP_sim = 2.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;

        // Robot / turret pose offsets
        public static final Distance kTurretOffsetX = Inches.of(0.0);
        public static final Distance kTurretOffsetY = Inches.of(0.0);
        public static final Distance kTurretOffsetZ = Inches.of(18.5);
        public static final Transform3d kRobotToTurretTransform =
                new Transform3d(
                        new Translation3d(
                                kTurretOffsetX.in(Meters), kTurretOffsetY.in(Meters), kTurretOffsetZ.in(Meters)),
                        new Rotation3d());

        // Limits / presets
        public static final Angle kHardLimitMax = Degrees.of(360);
        public static final Angle kHardLimitMin = Degrees.of(0);
        public static final Angle kSoftLimitMax = Degrees.of(90);
        public static final Angle kSoftLimitMin = Degrees.of(-90);
        // public static final Angle kSoftLimitMax = Degrees.of(360);
        // public static final Angle kSoftLimitMin = Degrees.of(0);
        public static final Angle kStartingPosition = Degrees.of(0);

        // Telemetry keys are centralized in TelemetryKeys
    }
}
