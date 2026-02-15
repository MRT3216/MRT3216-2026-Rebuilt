// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    // region Global Constants
    public static final RobotType robot = RobotType.COMPBOT;
    public static final boolean tuningMode = false;
    public static final double loopPeriodSecs = 0.02;
    public static final double loopPeriodWatchdogSecs = 0.2;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public enum RobotType {
        COMPBOT,
        SIMBOT
    }

    // endregion

    // region Physics Constants
    public static final class PhysicsConstants {
        private PhysicsConstants() {}

        public static final double kStandardGravity = 9.80665;
    }

    // endregion

    // region Robot Safety Constants
    public static final class RobotSafetyConstants {
        private RobotSafetyConstants() {}

        public static final double kLowBatteryVoltage = 11.0;
        public static final double kLowBatteryDisabledSecs = 2.0;
    }

    // endregion

    // region Drive / PathPlanner Constants
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

    // region Drive Control Constants
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

    // region PathPlanner Constants
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

    // region Communications / Phoenix Status Signal Constants
    public static final class CommsConstants {
        private CommsConstants() {}

        public static final double kDefaultStatusSignalHz = 50.0;
    }

    // endregion

    // region Shooter Constants (Flywheel)
    public static final class ShooterConstants {
        private ShooterConstants() {}

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
        public static final double kUpdateHz = 50.0;
        public static final String kMotorTelemetry = "FlywheelMotor";
        public static final String kMechTelemetry = "FlywheelMech";
    }

    // endregion

    // region Turret Constants
    public static final class TurretConstants {
        private TurretConstants() {}

        public static final double kGearing = 32.4;
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(60);
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0502269403);
        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 2.0;
        public static final double kMaxVelocityDegPerSec = 90.0;
        public static final double kMaxAccelDegPerSec2 = 45.0;
        public static final double kS = 0.1;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final Distance kTurretOffsetX = Inches.of(0.0);
        public static final Distance kTurretOffsetY = Inches.of(0.0);
        public static final Distance kTurretOffsetZ = Inches.of(18.5);
        public static final Transform3d kRobotToTurretTransform =
                new Transform3d(
                        new Translation3d(
                                kTurretOffsetX.in(Meters), kTurretOffsetY.in(Meters), kTurretOffsetZ.in(Meters)),
                        new Rotation3d());
        public static final Angle kHardLimitMax = Degrees.of(360);
        public static final Angle kHardLimitMin = Degrees.of(0);
        public static final Angle kSoftLimitMax = Degrees.of(350);
        public static final Angle kSoftLimitMin = Degrees.of(10);
        public static final Angle kStartingPosition = Degrees.of(180);
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
    }

    // endregion

    // region Spindexer Constants
    public static final class SpindexerConstants {
        private SpindexerConstants() {}

        public static final double kGearing = 5.0;
        public static final double kGearReduction = kGearing;
        public static final Distance kWheelDiameter = Inches.of(2);
        public static final Mass kWheelMass = Pounds.of(1);
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(30);
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.05;
        public static final double kV = 0.01;
        public static final double kA = 0.0;
        public static final String kMotorTelemetry = "SpindexerMotor";
        public static final String kMechTelemetry = "SpindexerMech";
    }

    // endregion

    // region Shooter Lookup Tables
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

        public static final double[][] PASS = {
            {1.0, 75.0, 54.0, 0.35},
            {5.5, 78.3, 45.0, 1.25},
        };
    }

    // endregion

    // region Kicker Constants
    public static final class KickerConstants {
        private KickerConstants() {}

        public static final Distance kWheelDiameter = Inches.of(2);
        public static final Mass kWheelMass = Pounds.of(1);
        public static final double kGearReduction = 1.0;
        public static final Current kStatorCurrentLimit = Amps.of(60);
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final String kMotorTelemetry = "KickerMotor";
        public static final String kMechTelemetry = "KickerMech";
    }

    // endregion

    // region Intake Constants
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
        public static final double kUpdateHz = 50.0;
        public static final String kMotorTelemetry = "IntakeRollersMotor";
        public static final String kMechTelemetry = "IntakeRollersMech";
    }

    // endregion

    // region Intake Arm Constants
    public static final class IntakePivotConstants {
        private IntakePivotConstants() {}

        public static final double kGearing = 125.0;
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(60);
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.05);
        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 2.0;
        public static final double kMaxVelocityDegPerSec = 90.0;
        public static final double kMaxAccelDegPerSec2 = 45.0;
        public static final double kS = 0.1;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final Angle kHardLimitMax = Degrees.of(360);
        public static final Angle kHardLimitMin = Degrees.of(0);
        public static final Angle kSoftLimitMax = Degrees.of(350);
        public static final Angle kSoftLimitMin = Degrees.of(10);
        public static final Angle kStartingPosition = Degrees.of(0);
        public static final String kMotorTelemetry = "IntakeArmMotor";
        public static final String kMechTelemetry = "IntakeArmMech";
    }

    // endregion

    // region Utility Methods
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
