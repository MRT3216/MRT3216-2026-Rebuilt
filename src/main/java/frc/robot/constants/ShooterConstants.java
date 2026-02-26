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
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    // Group flywheel-specific constants under a Flywheel section to improve organization.
    public static final class FlywheelConstants {
        private FlywheelConstants() {}
        // Mechanical
        public static final Distance kWheelDiameter = Inches.of(3);
        public static final Mass kWheelMass = Pounds.of(3);
        public static final double kGearReduction = 1.0;

        // Electrical / limits
        public static final Current kStatorCurrentLimit = Amps.of(80);

        // PID (velocity) - set to 0.0 for initial tuning
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward - set to 0.0 for initial tuning
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        // Simulation overrides - start at 0.0 until tuned
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.124;
        public static final double kA_sim = 0.1;
        public static final double kP_sim = 0.1;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;

        // Telemetry keys
        public static final String kFlywheelMotorTelemetry = "FlywheelMotor";
        public static final String kFlywheelMechTelemetry = "FlywheelMech";

        // Recommended target velocities / helper constants
        public static final AngularVelocity kFlywheelPrepAngularVelocity = RPM.of(2500);
        /** A lower spin-up/test velocity used for button-driven quick checks. */
        public static final AngularVelocity kFlywheelLowSpinAngularVelocity = RPM.of(1000.0);
        /** Fractional error margin (e.g. 0.02 == 2%) used to decide when the flywheel is "at speed" */
        public static final double kFlywheelAtSpeedError = 0.02;

        /**
         * Soft limits (human units) used for SysId / safety tooling. Defaults are conservative; tune on
         * robot.
         */
        public static final AngularVelocity kSoftLimitMax = RPM.of(4500.0);

        public static final AngularVelocity kSoftLimitMin = RPM.of(0.0);

        // Motion limits for trajectory generation (used by closed-loop/trapezoidal profiles)
        public static final AngularVelocity kMaxVelocity = RPM.of(5000.0);
        // Acceleration used for trapezoidal profiling. Units: deg/s^2 (DegreesPerSecondPerSecond)
        public static final AngularAcceleration kMaxAccel = DegreesPerSecondPerSecond.of(6000.0);

        /**
         * Early-exit threshold (meters). If iterative refinement changes lead distance by less than
         * this amount between passes, stop iterating early.
         */
        public static final Distance kRefinementConvergenceEpsilon = Meters.of(0.01); // 1 cm

        /** Duration to run the clear routine while the flywheel spins up (seconds). */
        public static final double kClearDurationSecs = 2;

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
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // PID / Feedforward
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        // Simulation-specific feedforward / PID defaults for Spindexer
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.624;
        public static final double kA_sim = 0.0;
        public static final double kP_sim = 0.01;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;

        // Telemetry keys
        public static final String kSpindexerMotorTelemetry = "SpindexerMotor";
        public static final String kSpindexerMechTelemetry = "SpindexerMech";

        // Recommended / helper velocities
        public static final AngularVelocity kSpindexerTargetAngularVelocity = RPM.of(600.0);
        // Default clear velocity (negative to reverse) used to clear jams
        public static final AngularVelocity kSpindexerClearAngularVelocity = RPM.of(-10.0);

        /**
         * Soft limits (human units) used for SysId / safety tooling. Defaults are conservative; tune on
         * robot.
         */
        public static final AngularVelocity kSoftLimitMax = RPM.of(1000.0);

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
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // PID
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        // Simulation variants
        public static final double kS_sim = 0;
        public static final double kV_sim = 0.1045;
        public static final double kA_sim = 0;
        public static final double kP_sim = 0.001;
        public static final double kI_sim = 0;
        public static final double kD_sim = 0;

        // Telemetry
        public static final String kKickerMotorTelemetry = "KickerMotor";
        public static final String kKickerMechTelemetry = "KickerMech";

        // Recommended velocities
        public static final AngularVelocity kKickerTargetAngularVelocity = RPM.of(2000.0);
        public static final AngularVelocity kKickerClearAngularVelocity = RPM.of(-100.0);

        /**
         * Soft limits (human units) used for SysId / safety tooling. Defaults are conservative; tune on
         * robot.
         */
        public static final AngularVelocity kSoftLimitMax = RPM.of(4000.0);

        public static final AngularVelocity kSoftLimitMin = RPM.of(0.0);

        // Motion limits for trajectory generation
        public static final AngularVelocity kMaxVelocity = RPM.of(4000.0);
        // Acceleration used for trapezoidal profiling. Units: deg/s^2
        public static final AngularAcceleration kMaxAccel = DegreesPerSecondPerSecond.of(2400.0);

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
        public static final Distance kLength = Inches.of(10.0);

        /** Approximate mass of the hood assembly. */
        public static final Mass kMass = Pounds.of(5.0);

        // Motor wiring
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(30);

        // PID
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward (Arm-style)
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        /** Gravity/feedforward constant used by arm-style feedforward (ArmFeedforward). */
        public static final double kG = 0.0;

        // Simulation-specific feedforward / PID defaults for Hood (tuned for sim)
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.0;
        public static final double kA_sim = 0.0;
        /** Simulation gravity/feedforward term for ArmFeedforward in sim. */
        public static final double kG_sim = 0.0;

        public static final double kP_sim = 0.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.5;

        // Motion limits
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(90.0);
        // Acceleration used for trapezoidal profiling (deg/s^2)
        public static final AngularAcceleration kMaxAccel = DegreesPerSecondPerSecond.of(90.0);

        // Limits / presets
        public static final Angle kHardLimitMax = Degrees.of(56);
        public static final Angle kHardLimitMin = Degrees.of(23);
        public static final Angle kSoftLimitMax = Degrees.of(54);
        public static final Angle kSoftLimitMin = Degrees.of(25);
        public static final Angle kStartingPosition = Degrees.of(25);

        // Telemetry
        public static final String kHoodMotorTelemetry = "HoodMotor";
        public static final String kHoodMechTelemetry = "HoodMech";
        /** Allowed absolute position error for hood angle comparisons (degrees). */
        public static final Angle kPositionTolerance = Degrees.of(1.0);

        /**
         * Returns a preconfigured ArmFeedforward instance for the Hood arm. Uses the ordering
         * ArmFeedforward(ks, kg, kv) to match arm semantics.
         */
        public static ArmFeedforward armFeedforward() {
            return new ArmFeedforward(kS, kG, kV, kA);
        }

        /** Simulation variant of the Hood arm feedforward. */
        public static ArmFeedforward armFeedforwardSim() {
            return new ArmFeedforward(kS_sim, kG_sim, kV_sim, kA_sim);
        }
    }

    public static final class TurretConstants {
        private TurretConstants() {}
        // Mechanical
        public static final double kGearing = 32.4;

        // Motor wiring
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(60);

        // Inertia / dynamics
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0502269403);

        // PID
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Motion limits
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(90.0);
        // Acceleration used for trapezoidal profiling (deg/s^2)
        public static final AngularAcceleration kMaxAccel = DegreesPerSecondPerSecond.of(90.0);

        // Feedforward
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        // Simulation-specific feedforward / PID defaults (tuned for sim)
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 2;
        public static final double kA_sim = 0.2;
        // Simulation-tuned PID defaults (reduced to avoid oscillation in sim)
        public static final double kP_sim = 2.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;

        // Robot / turret pose offsets
        public static final Distance kTurretOffsetX = Inches.of(0.0);
        public static final Distance kTurretOffsetY = Inches.of(0.0);
        public static final Distance kTurretOffsetZ = Inches.of(18.5);
        public static final edu.wpi.first.math.geometry.Transform3d kRobotToTurretTransform =
                new edu.wpi.first.math.geometry.Transform3d(
                        new edu.wpi.first.math.geometry.Translation3d(
                                kTurretOffsetX.in(Meters), kTurretOffsetY.in(Meters), kTurretOffsetZ.in(Meters)),
                        new edu.wpi.first.math.geometry.Rotation3d());

        // Limits / presets
        public static final Angle kHardLimitMax = Degrees.of(120);
        public static final Angle kHardLimitMin = Degrees.of(-120);
        public static final Angle kSoftLimitMax = Degrees.of(90);
        public static final Angle kSoftLimitMin = Degrees.of(-90);
        public static final Angle kStartingPosition = Degrees.of(0);
        /** Allowed absolute position error for turret angle comparisons (degrees). */
        public static final Angle kPositionTolerance = Degrees.of(1.0);

        // Shooting helpers
        // public static final Distance kMinShootingDistance = Meters.of(1.5);
        // public static final Distance kMaxShootingDistance = Meters.of(12.0);
        // public static final LinearVelocity kBaseVel = InchesPerSecond.of(300);
        // public static final double kVelMultiplier = 0.5;
        // public static final double kVelPower = 1.2;
        // public static final Distance kDistanceAboveFunnel = Inches.of(12.0);
        // public static final Distance kFunnelRadius = Inches.of(24.0);
        // public static final Distance kFunnelHeight = Inches.of(104.0);

        // Telemetry
        public static final String kTurretMotorTelemetry = "TurretMotor";
        public static final String kTurretMechTelemetry = "TurretMech";

        /** Gearing used specifically for external encoder wiring (motor:mechanism). */
        public static final double kEncoderGearing = 1.0;

        /**
         * Encoder ratios for EasyCRT / external-encoder wiring. These represent encoder rotations per
         * one mechanism rotation (encoder_rotations / mech_rotation). Use `withEncoderRatios(...)` or
         * feed into gearing helpers when configuring EasyCRT or external encoder `MechanismGearing`.
         */
        public static final double kEncoder1RotPerMechRot =
                1.0; // Spark-mounted absolute encoder (default 1:1)

        public static final double kEncoder2RotPerMechRot =
                1.0; // RoboRIO PWM absolute encoder (default 1:1)

        // EasyCRT configuration constants (simple teeth counts)
        /** Encoder 1 gear teeth: driver (encoder pinion) */
        public static final int kEasyCrtEncoder1DriverTeeth = 13;
        /** Driven teeth on the turret gear (shared) */
        public static final int kTurretDrivenTeeth = 90;
        /** Motor pinion teeth (driver) used for mechanism gearing */
        public static final int kTurretMotorDriverTeeth = 10;

        /** Mechanism search range for EasyCRT (rotations) */
        public static final Angle kEasyCrtMechanismRangeMin = Rotations.of(0.0);

        public static final Angle kEasyCrtMechanismRangeMax = Rotations.of(1.2);

        /** Absolute encoder inversion flags used by EasyCRT */
        public static final boolean kEasyCrtAbs1Inverted = false;

        public static final boolean kEasyCrtAbs2Inverted = false;

        /** Match tolerance for EasyCRT (rotations). Tune during bring-up. */
        public static final Angle kEasyCrtMatchTolerance = Rotations.of(0.06);

        /* EasyCRT bring-up tuning knobs */
        public static final int kEasyCrtMaxAttempts = 10;
        /** Periodic cycles between EasyCRT attempts (e.g., 10 at 50Hz ≈ 0.2s). */
        public static final int kEasyCrtPeriodicSpacing = 10;

        /* EasyCRT telemetry keys */
        public static final String kEasyCrtStatusKey = "EasyCRT/Status";
        public static final String kEasyCrtLastErrorRotKey = "EasyCRT/LastErrorRot";
        public static final String kEasyCrtIterationsKey = "EasyCRT/Iterations";
        public static final String kEasyCrtRecPairKey = "EasyCRT/RecPair";
        public static final String kEasyCrtCoverageKey = "EasyCRT/UniqueCoverage";
        /** Telemetry key for the final solved mechanism angle when EasyCRT succeeds. */
        public static final String kEasyCrtSolvedAngleKey = "EasyCRT/SolvedAngle";

        /**
         * When true, log solver iterations and last-error on successful solves. Useful during tuning.
         */
        public static final boolean kEasyCrtLogOnSuccess = true;

        // Simulation-only CRT gear recommender constraints
        public static final double kCrtGearRecCoverage = 1.2;
        public static final int kCrtGearRecMinTeeth = 15;
        public static final int kCrtGearRecMaxTeeth = 45;
        public static final int kCrtGearRecMaxCompoundTeeth = 30;

        /** Returns a SimpleMotorFeedforward configured for turret controllers. */
        public static SimpleMotorFeedforward motorFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the turret feedforward. */
        public static SimpleMotorFeedforward motorFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }
    }
}
