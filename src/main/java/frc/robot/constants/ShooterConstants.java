// Shooter constants extracted from `Constants.java` for easier maintenance and smaller file churn.
// @source: measured and tuned by Littleton Robotics team (2025-2026)
package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

/** Flywheel shooter constants: wheel geometry, control gains, and velocities. */
public final class ShooterConstants {
    private ShooterConstants() {}

    // Group flywheel-specific constants under a Flywheel section to improve organization.
    public static final class FlywheelConstants {
        private FlywheelConstants() {}

        public static final Distance kWheelDiameter = Inches.of(3);
        public static final Mass kWheelMass = Pounds.of(3);
        public static final double kGearReduction = 1.0;
        public static final Current kStatorCurrentLimit = Amps.of(80);
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.15;
        public static final double kV = 0.00207;
        public static final double kA = 0.0001;

        // SIM variants (tuned for smoother/less-friction simulation)
        public static final double kS_sim = 0.10;
        public static final double kV_sim = 0.0018;
        public static final double kA_sim = 0.00008;
        public static final double kP_sim = 0.6;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        public static final double kUpdateHz = 50.0;
        public static final String kFlywheelMotorTelemetry = "FlywheelMotor";
        public static final String kFlywheelMechTelemetry = "FlywheelMech";

        // Recommended test target velocity for coordinated shooting (units-aware)
        public static final AngularVelocity kFlywheelTargetAngularVelocity = RPM.of(3000.0);
        /** A lower spin-up/test velocity used for button-driven quick checks. */
        public static final AngularVelocity kFlywheelLowSpinAngularVelocity = RPM.of(1000.0);
        /** Fractional error margin (e.g. 0.02 == 2%) used to decide when the flywheel is "at speed" */
        public static final double kFlywheelAtSpeedError = 0.02;
        /**
         * Early-exit threshold (meters). If iterative refinement changes lead distance by less than
         * this amount between passes, stop iterating early.
         */
        public static final Distance kRefinementConvergenceEpsilon = Meters.of(0.01); // 1 cm

        // (No SIM-specific overrides: use the same tuned constants in both environments.)
        /** Duration to run the clear routine while the flywheel spins up (seconds). */
        public static final double kClearDurationSecs = 0.5;
    }

    // ---------------------------------------------------------------------
    // Additional shooter-adjacent constants previously nested in Constants.java
    // These were moved here to centralize shooter subsystem tuning and reduce churn
    // in the large `Constants.java` file.
    // ---------------------------------------------------------------------

    public static final class SpindexerConstants {
        private SpindexerConstants() {}

        public static final double kGearing = 5.0;
        /** Backwards-compatible alias used by some subsystems. */
        public static final double kGearReduction = kGearing;

        /** Roller physical size used by FlyWheel configs. */
        public static final Distance kWheelDiameter = Inches.of(2);

        /** Roller mass used by FlyWheel configs. */
        public static final Mass kWheelMass = Pounds.of(0.5);

        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(30);
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.05;
        public static final double kV = 0.01;
        public static final double kA = 0.0;
        // Simulation-specific feedforward / PID defaults for Spindexer
        public static final double kS_sim = 0.03;
        public static final double kV_sim = 0.008;
        public static final double kA_sim = kA;
        public static final double kP_sim = 0.35;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        public static final String kSpindexerMotorTelemetry = "SpindexerMotor";
        public static final String kSpindexerMechTelemetry = "SpindexerMech";
        // Recommended target velocity for the spindexer (units-aware)
        public static final AngularVelocity kSpindexerTargetAngularVelocity = RPM.of(2000.0);
        // Default clear velocity (negative to reverse) used to clear jams
        public static final AngularVelocity kSpindexerClearAngularVelocity = RPM.of(-100.0);
    }

    public static final class KickerConstants {
        private KickerConstants() {}

        public static final Distance kWheelDiameter = Inches.of(2.5);
        public static final Mass kWheelMass = Pounds.of(1);
        public static final double kGearReduction = 1.0;
        public static final Current kStatorCurrentLimit = Amps.of(40);

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.05;
        public static final double kV = 0.001;
        public static final double kA = 0.0001;

        public static final double kS_sim = kS;
        public static final double kV_sim = kV;
        public static final double kA_sim = kA;
        public static final double kP_sim = kP;
        public static final double kI_sim = kI;
        public static final double kD_sim = kD;

        public static final String kKickerMotorTelemetry = "KickerMotor";
        public static final String kKickerMechTelemetry = "KickerMech";

        public static final AngularVelocity kKickerTargetAngularVelocity = RPM.of(2000.0);
        public static final AngularVelocity kKickerClearAngularVelocity = RPM.of(-100.0);
    }

    public static final class HoodConstants {
        private HoodConstants() {}

        /** Gearing from motor to hood pivot (ratio). */
        public static final double kGearing = 30.0;

        /** Physical arm length from pivot to hood center (useful for dynamics) */
        public static final Distance kLength = Inches.of(6.0);

        /** Approximate mass of the hood assembly. */
        public static final Mass kMass = Pounds.of(1.0);

        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(30);

        /* PID */
        public static final double kP = 6.0;
        public static final double kI = 0.0;
        public static final double kD = 1.0;

        /* Feedforward (Arm-style) */
        public static final double kS = 0.05;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        // Simulation-specific feedforward / PID defaults for Hood (tuned for sim)
        public static final double kS_sim = 0.03;
        public static final double kV_sim = 0.09;
        public static final double kA_sim = 0.008;
        public static final double kP_sim = 4.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.5;

        public static final AngularVelocity kMaxVelocity =
                edu.wpi.first.units.Units.DegreesPerSecond.of(90.0);
        public static final edu.wpi.first.units.measure.AngularAcceleration kMaxAccelDegPerSec2 =
                edu.wpi.first.units.Units.DegreesPerSecondPerSecond.of(180.0);

        public static final edu.wpi.first.units.measure.Angle kHardLimitMax =
                edu.wpi.first.units.Units.Degrees.of(90);
        public static final edu.wpi.first.units.measure.Angle kHardLimitMin =
                edu.wpi.first.units.Units.Degrees.of(0);
        public static final edu.wpi.first.units.measure.Angle kSoftLimitMax =
                edu.wpi.first.units.Units.Degrees.of(85);
        public static final edu.wpi.first.units.measure.Angle kSoftLimitMin =
                edu.wpi.first.units.Units.Degrees.of(5);
        public static final edu.wpi.first.units.measure.Angle kStartingPosition =
                edu.wpi.first.units.Units.Degrees.of(0);

        public static final String kHoodMotorTelemetry = "HoodMotor";
        public static final String kHoodMechTelemetry = "HoodMech";
        /** Allowed absolute position error for hood angle comparisons (degrees). */
        public static final edu.wpi.first.units.measure.Angle kPositionTolerance =
                edu.wpi.first.units.Units.Degrees.of(1.0);
    }

    public static final class TurretConstants {
        private TurretConstants() {}

        public static final double kGearing = 32.4;
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(60);
        public static final edu.wpi.first.units.measure.MomentOfInertia kMOI =
                edu.wpi.first.units.Units.KilogramSquareMeters.of(0.0502269403);
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final AngularVelocity kMaxVelocity =
                edu.wpi.first.units.Units.DegreesPerSecond.of(90.0);
        public static final edu.wpi.first.units.measure.AngularAcceleration kMaxAccelDegPerSec2 =
                edu.wpi.first.units.Units.DegreesPerSecondPerSecond.of(45.0);
        public static final double kS = 0.1;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        // Simulation-specific feedforward / PID defaults (tuned for sim)
        public static final double kS_sim = 2.0;
        public static final double kV_sim = 0.0;
        public static final double kA_sim = 0.0;
        // Simulation-tuned PID defaults (reduced to avoid oscillation in sim)
        public static final double kP_sim = 0.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        public static final Distance kTurretOffsetX = Inches.of(0.0);
        public static final Distance kTurretOffsetY = Inches.of(0.0);
        public static final Distance kTurretOffsetZ = Inches.of(18.5);
        public static final edu.wpi.first.math.geometry.Transform3d kRobotToTurretTransform =
                new edu.wpi.first.math.geometry.Transform3d(
                        new edu.wpi.first.math.geometry.Translation3d(
                                kTurretOffsetX.in(Meters), kTurretOffsetY.in(Meters), kTurretOffsetZ.in(Meters)),
                        new edu.wpi.first.math.geometry.Rotation3d());
        public static final edu.wpi.first.units.measure.Angle kHardLimitMax =
                edu.wpi.first.units.Units.Degrees.of(180);
        public static final edu.wpi.first.units.measure.Angle kHardLimitMin =
                edu.wpi.first.units.Units.Degrees.of(-180);
        public static final edu.wpi.first.units.measure.Angle kSoftLimitMax =
                edu.wpi.first.units.Units.Degrees.of(170);
        public static final edu.wpi.first.units.measure.Angle kSoftLimitMin =
                edu.wpi.first.units.Units.Degrees.of(-170);
        public static final edu.wpi.first.units.measure.Angle kStartingPosition =
                edu.wpi.first.units.Units.Degrees.of(0);
        /** Allowed absolute position error for turret angle comparisons (degrees). */
        public static final edu.wpi.first.units.measure.Angle kPositionTolerance =
                edu.wpi.first.units.Units.Degrees.of(1.0);

        public static final Distance kMinShootingDistance = Meters.of(1.5);
        public static final Distance kMaxShootingDistance = Meters.of(12.0);
        public static final edu.wpi.first.units.measure.LinearVelocity kBaseVel =
                edu.wpi.first.units.Units.InchesPerSecond.of(300);
        public static final double kVelMultiplier = 0.5;
        public static final double kVelPower = 1.2;
        public static final Distance kDistanceAboveFunnel = Inches.of(12.0);
        public static final Distance kFunnelRadius = Inches.of(24.0);
        public static final Distance kFunnelHeight = Inches.of(104.0);
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
        public static final edu.wpi.first.units.measure.Angle kEasyCrtMechanismRangeMin =
                edu.wpi.first.units.Units.Rotations.of(0.0);

        public static final edu.wpi.first.units.measure.Angle kEasyCrtMechanismRangeMax =
                edu.wpi.first.units.Units.Rotations.of(1.2);

        /** Absolute encoder inversion flags used by EasyCRT */
        public static final boolean kEasyCrtAbs1Inverted = false;

        public static final boolean kEasyCrtAbs2Inverted = false;

        // Simulation-only CRT gear recommender constraints
        public static final double kCrtGearRecCoverage = 1.2;
        public static final int kCrtGearRecMinTeeth = 15;
        public static final int kCrtGearRecMaxTeeth = 45;
        public static final int kCrtGearRecMaxCompoundTeeth = 30;
    }
}
