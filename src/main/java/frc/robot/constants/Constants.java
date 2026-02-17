// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Central repository of robot constants. Keep this file tidy: avoid duplicate nested classes and
 * prefer Javadoc over short inline comment headings.
 */
public final class Constants {
    /** Selected robot hardware profile used at runtime (COMPBOT or SIMBOT). */
    public static final RobotType robot = RobotType.COMPBOT;

    /** Whether the project is running in tuning mode (enables extra telemetry/tunables). */
    public static final boolean tuningMode = false;

    /** Main control loop period (seconds). */
    public static final double loopPeriodSecs = 0.02;

    /** Watchdog period for loop monitoring (seconds). */
    public static final double loopPeriodWatchdogSecs = 0.2;

    /**
     * Currently active runtime mode (REAL, SIM, REPLAY).
     *
     * <p>By default this is inferred from WPILib's {@link RobotBase#isReal()}. To force a specific
     * mode for testing or CI, set {@link #FORCE_MODE} to the desired value below.
     */
    public static final Mode FORCE_MODE = null; // set to Mode.SIM or Mode.REAL to override

    public static final Mode currentMode =
            (FORCE_MODE != null) ? FORCE_MODE : (RobotBase.isReal() ? Mode.REAL : Mode.SIM);

    /** Operation modes for the robot (simulation, real robot, etc.). */
    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    /** Robot hardware profile selection used to pick IO implementations. */
    public enum RobotType {
        COMPBOT,
        SIMBOT
    }

    // region Shooter
    public static final class ShooterConstants {
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
        public static final String kMotorTelemetry = "FlywheelMotor";
        public static final String kMechTelemetry = "FlywheelMech";
        // Recommended test target velocity for coordinated shooting (units-aware)
        public static final AngularVelocity kTargetFlywheel = RPM.of(3000.0);
        // (No SIM-specific overrides: use the same tuned constants in both environments.)
        /** Duration to run the clear routine while the flywheel spins up (seconds). */
        public static final double kClearDurationSecs = 0.5;
    }

    // endregion

    // region Kicker
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

        public static final String kMotorTelemetry = "KickerMotor";
        public static final String kMechTelemetry = "KickerMech";

        public static final AngularVelocity kTargetVelocity = RPM.of(2000.0);
        public static final AngularVelocity kClearVelocity = RPM.of(-100.0);
    }

    // endregion

    // region Turret
    public static final class TurretConstants {
        private TurretConstants() {}

        // Motor gear ratio: 10 teeth motor pinion to 90 teeth turret driven = 9:1
        public static final double kGearing = 9.0;
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(60);
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0502269403);
        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 2.0;
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(90.0);
        public static final AngularAcceleration kMaxAccelDegPerSec2 =
                DegreesPerSecondPerSecond.of(45.0);
        public static final double kS = 0.1;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        // Simulation-specific feedforward / PID defaults (tuned for sim)
        public static final double kS_sim = 0.05;
        public static final double kV_sim = 0.09;
        public static final double kA_sim = 0.008;
        // Simulation-tuned PID defaults (reduced to avoid oscillation in sim)
        public static final double kP_sim = 0.8;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.2;
        public static final Distance kTurretOffsetX = Inches.of(0.0);
        public static final Distance kTurretOffsetY = Inches.of(0.0);
        public static final Distance kTurretOffsetZ = Inches.of(18.5);
        public static final Transform3d kRobotToTurretTransform =
                new Transform3d(
                        new Translation3d(
                                kTurretOffsetX.in(Meters), kTurretOffsetY.in(Meters), kTurretOffsetZ.in(Meters)),
                        new Rotation3d());
        public static final Angle kHardLimitMax = Degrees.of(180);
        public static final Angle kHardLimitMin = Degrees.of(-180);
        public static final Angle kSoftLimitMax = Degrees.of(170);
        public static final Angle kSoftLimitMin = Degrees.of(-170);
        public static final Angle kStartingPosition = Degrees.of(0);
        public static final Distance kMinShootingDistance = Meters.of(1.5);
        public static final Distance kMaxShootingDistance = Meters.of(12.0);
        public static final LinearVelocity kBaseVel = InchesPerSecond.of(300);
        public static final double kVelMultiplier = 0.5;
        public static final double kVelPower = 1.2;
        public static final Distance kDistanceAboveFunnel = Inches.of(12.0);
        public static final Distance kFunnelRadius = Inches.of(24.0);
        public static final Distance kFunnelHeight = Inches.of(104.0);
        public static final String kMotorTelemetry = "TurretMotor";
        public static final String kMechTelemetry = "TurretMech";
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
        public static final Angle kEasyCrtMechanismRangeMin = Units.Rotations.of(0.0);

        public static final Angle kEasyCrtMechanismRangeMax = Units.Rotations.of(1.2);

        /** Absolute encoder inversion flags used by EasyCRT */
        public static final boolean kEasyCrtAbs1Inverted = false;

        public static final boolean kEasyCrtAbs2Inverted = false;

        // Simulation-only CRT gear recommender constraints
        public static final double kCrtGearRecCoverage = 1.2;
        public static final int kCrtGearRecMinTeeth = 15;
        public static final int kCrtGearRecMaxTeeth = 45;
        public static final int kCrtGearRecMaxCompoundTeeth = 30;
    }

    // endregion

    // region Drive
    public static final class DriveConstants {
        private DriveConstants() {}

        public static final double kRobotMassKg = 74.088;
        public static final double kRobotMOI = 6.883;
        public static final double kWheelCoef = 1.2;
        public static final double kOdometryFreqNetworkFD = 250.0;
        public static final double kOdometryFreqCAN = 100.0;
        public static final double kDefaultMotionMagicCruiseVelocity = 100.0;
        public static final double kMotionMagicAccelWindowSec = 0.100;
    }

    // endregion

    // region Communications
    public static final class CommsConstants {
        private CommsConstants() {}

        public static final double kDefaultStatusSignalHz = 50.0;
    }

    // endregion

    // region PathPlanner
    public static final class PathPlannerConstants {
        private PathPlannerConstants() {}

        public static final double kTranslationP = 5.0;
        public static final double kTranslationI = 0.0;
        public static final double kTranslationD = 0.0;
        public static final double kRotationP = 5.0;
        public static final double kRotationI = 0.0;
        public static final double kRotationD = 0.0;
    }

    // endregion

    // region DriveControl
    public static final class DriveControlConstants {
        private DriveControlConstants() {}

        public static final double kDeadband = 0.1;
        public static final double kAngleKP = 5.0;
        public static final double kAngleKD = 0.4;
        public static final double kAngleMaxVelocity = 8.0;
        public static final double kAngleMaxAcceleration = 20.0;
        public static final double kFFStartDelay = 2.0;
        public static final double kFFRampRate = 0.1;
        public static final double kWheelRadiusMaxVelocity = 0.25;
        public static final double kWheelRadiusRampRate = 0.05;
    }

    // endregion

    // region RobotSafety
    public static final class RobotSafetyConstants {
        private RobotSafetyConstants() {}

        public static final double kLowBatteryVoltage = 11.0;
        public static final double kLowBatteryDisabledSecs = 2.0;
    }

    // endregion

    // region Physics
    public static final class PhysicsConstants {
        private PhysicsConstants() {}

        public static final Mass kWheelMass = Pounds.of(1);
        public static final double kGearReduction = 1.0;
        public static final Current kStatorCurrentLimit = Amps.of(60);
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        // Simulation-specific feedforward / PID defaults for Kicker
        public static final double kS_sim = kS;
        public static final double kV_sim = kV;
        public static final double kA_sim = kA;
        public static final double kP_sim = 0.7;
        public static final double kI_sim = kI;
        public static final double kD_sim = kD;
        public static final String kMotorTelemetry = "KickerMotor";
        public static final String kMechTelemetry = "KickerMech";
        // Recommended target velocity for the kicker (units-aware)
        public static final AngularVelocity kTargetVelocity = RPM.of(2000.0);
        // Default clear velocity (negative to reverse) used to clear jams
        public static final AngularVelocity kClearVelocity = RPM.of(-100.0);
    }

    // endregion

    // region Spindexer
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
        public static final String kMotorTelemetry = "SpindexerMotor";
        public static final String kMechTelemetry = "SpindexerMech";
        // Recommended target velocity for the spindexer (units-aware)
        public static final AngularVelocity kTargetVelocity = RPM.of(2000.0);
        // Default clear velocity (negative to reverse) used to clear jams
        public static final AngularVelocity kClearVelocity = RPM.of(-100.0);
    }

    // endregion

    // region Hood
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

        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(90.0);
        public static final AngularAcceleration kMaxAccelDegPerSec2 =
                DegreesPerSecondPerSecond.of(180.0);

        public static final Angle kHardLimitMax = Degrees.of(90);
        public static final Angle kHardLimitMin = Degrees.of(0);
        public static final Angle kSoftLimitMax = Degrees.of(85);
        public static final Angle kSoftLimitMin = Degrees.of(5);
        public static final Angle kStartingPosition = Degrees.of(0);

        public static final String kMotorTelemetry = "HoodMotor";
        public static final String kMechTelemetry = "HoodMech";
    }

    // endregion

    // region ShooterLookupTables
    public static final class ShooterLookupTables {
        private ShooterLookupTables() {}

        public static final double[][] HUB = {
            {1.0, 80.0, 75.0, 0.45},
            {2.0, 82.5, 72.0, 0.65},
            {3.0, 85.0, 68.0, 0.85},
            {4.0, 90.0, 65.0, 1.05},
            {5.0, 95.0, 62.0, 1.25},
            {6.0, 105.0, 60.0, 1.45},
        };
        public static final double[][] PASS = {{1.0, 75.0, 54.0, 0.35}, {5.5, 78.3, 45.0, 1.25}};
    }

    // endregion

    // region Intake
    public static final class IntakeConstants {
        private IntakeConstants() {}

        public static final Distance kWheelDiameter = Inches.of(3.5);
        public static final Mass kWheelMass = Pounds.of(2);
        public static final double kGearReduction = 1.0;
        public static final Current kStatorCurrentLimit = Amps.of(80);
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.15;
        public static final double kV = 0.00207;
        public static final double kA = 0.0001;
        // SIM variants (tuned for sim: reduce P and feedforward a bit)
        public static final double kS_sim = 0.12;
        public static final double kV_sim = 0.001656;
        public static final double kA_sim = 0.00008;
        public static final double kP_sim = 0.35;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        public static final double kUpdateHz = 50.0;
        public static final String kMotorTelemetry = "IntakeRollersMotor";
        public static final String kMechTelemetry = "IntakeRollersMech";
    }

    // endregion

    // region IntakePivot
    public static final class IntakePivotConstants {
        private IntakePivotConstants() {}

        public static final double kGearing = 30.0;
        public static final Distance kLength = Inches.of(11.5);
        public static final Mass kMass = Pounds.of(6.4);
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(60);
        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 2.0;
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(20.0);
        public static final AngularAcceleration kMaxAccelDegPerSec2 =
                DegreesPerSecondPerSecond.of(20.0);
        public static final double kS = 0.1;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        // SIM variants (tuned for sim)
        public static final double kS_sim = 0.05;
        public static final double kV_sim = 0.09;
        public static final double kA_sim = 0.008;
        public static final double kP_sim = 6.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 1.0;
        public static final Angle kHardLimitMax = Degrees.of(360);
        public static final Angle kHardLimitMin = Degrees.of(0);
        public static final Angle kSoftLimitMax = Degrees.of(350);
        public static final Angle kSoftLimitMin = Degrees.of(10);
        public static final Angle kStartingPosition = Degrees.of(0);
        public static final String kMotorTelemetry = "IntakeArmMotor";
        public static final String kMechTelemetry = "IntakeArmMech";
        /** Gearing used specifically for external encoder wiring (motor:mechanism). */
        public static final double kEncoderGearing = 1.0;
    }

    // endregion

    // region Utility
    private static boolean disableHAL = false;

    public static void setDisableHAL() {
        setDisableHAL(true);
    }

    public static void setDisableHAL(boolean disable) {
        disableHAL = disable;
    }

    public static boolean isDisableHAL() {
        return disableHAL;
    }

    public static class CheckDeploy {
        public static void main(String... args) {
            if (robot == RobotType.SIMBOT) {
                System.err.println("Cannot deploy, invalid robot selected: " + robot);
                System.exit(1);
            }
        }
    }

    public static class CheckPullRequest {
        public static void main(String... args) {
            if (robot != RobotType.COMPBOT || tuningMode) {
                System.err.println("Do not merge, non-default constants are configured.");
                System.exit(1);
            }
        }
    }

    // endregion

}
