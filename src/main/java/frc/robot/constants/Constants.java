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

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose.
 */
public final class Constants {
    public static final RobotType robot = RobotType.COMPBOT;
    public static final boolean tuningMode = false;

    public static final double loopPeriodSecs = 0.02;
    public static final double loopPeriodWatchdogSecs = 0.2;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public enum RobotType {
        COMPBOT,
        SIMBOT
    }

    // ==========================================
    // Shooter Constants (Flywheel)
    // ==========================================
    public static final class ShooterConstants {
        // Physical Constants
        public static final Distance kWheelDiameter = Inches.of(3);
        public static final Mass kWheelMass = Pounds.of(3);
        public static final double kGearReduction = 1.0;

        // Current Limits
        public static final Current kStatorCurrentLimit = Amps.of(80);

        // Feedback (PID) Constants
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Theoretical Feedforward (Kraken X60 FOC)
        public static final double kS = 0.15; // Volts to break friction
        public static final double kV = 0.00207; // Volts per RPM
        public static final double kA = 0.0001; // Volts per RPM^2

        // Telemetry Update Frequency
        public static final double kUpdateHz = 50.0;
    }

    // ==========================================
    // Turret Constants
    // ==========================================
    public static final class TurretConstants {
        private TurretConstants() {}

        // Hardware & Motor Configuration
        public static final int kTurretMotorId = 53;
        public static final double kGearing = 32.4000;
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(60);
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0502269403);

        // PID / Motion Profiling (Trapezoidal Limits)
        // High-P preference for precise geared control
        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 2.0;
        public static final double kMaxVelocityDegPerSec = 90.0;
        public static final double kMaxAccelDegPerSec2 = 45.0;

        // Feedforward Constants
        public static final double kS = 0.1; // Static friction voltage
        public static final double kV = 0.12; // Velocity feedforward
        public static final double kA = 0.01; // Acceleration feedforward

        // Geometry & Transforms
        // Physical offset from robot origin (center-floor) to turret pivot point
        public static final Distance kTurretOffsetX = Inches.of(0.0);
        public static final Distance kTurretOffsetY = Inches.of(0.0);
        public static final Distance kTurretOffsetZ = Inches.of(18.5);

        public static final Transform3d kRobotToTurretTransform =
                new Transform3d(
                        new Translation3d(
                                kTurretOffsetX.in(Meters), kTurretOffsetY.in(Meters), kTurretOffsetZ.in(Meters)),
                        new Rotation3d());

        // Mechanism Limits
        public static final Angle kHardLimitMax = Degrees.of(180);
        public static final Angle kHardLimitMin = Degrees.of(-180);
        public static final Angle kSoftLimitMax = Degrees.of(170);
        public static final Angle kSoftLimitMin = Degrees.of(-170);
        public static final Angle kStartingPosition = Degrees.of(0);

        // Validation Ranges
        public static final Distance kMinShootingDistance = Meters.of(1.5);
        public static final Distance kMaxShootingDistance = Meters.of(12.0);

        // Physics & Ramp Constants (TurretCalculator)
        // v = BaseVel + (Multiplier * distance ^ Power)
        public static final LinearVelocity kBaseVel = InchesPerSecond.of(300);
        public static final double kVelMultiplier = 0.5;
        public static final double kVelPower = 1.2;

        // Clearance for Funnel Obstacle
        public static final Distance kDistanceAboveFunnel = Inches.of(12.0);
        public static final Distance kFunnelRadius = Inches.of(24.0);
        public static final Distance kFunnelHeight = Inches.of(104.0);

        // Telemetry Labels
        public static final String kMotorTelemetry = "TurretMotor";
        public static final String kMechTelemetry = "TurretMech";
    }

    // ==========================================
    // Drive / PathPlanner Constants
    // ==========================================
    public static final class DriveConstants {
        private DriveConstants() {}

        // Robot physical properties used by PathPlanner
        // Mass of the robot in kilograms
        public static final double kRobotMassKg = 74.088;
        // Moment of inertia used by PathPlanner
        public static final double kRobotMOI = 6.883;
        // Wheel coefficient used by PathPlanner module config
        public static final double kWheelCoef = 1.2;

        // Telemetry / tuning
        public static final double kOdometryFreqNetworkFD = 250.0;
        public static final double kOdometryFreqCAN = 100.0;

        // MotionMagic / low-level motion defaults used by TalonFX-based module implementations
        // Cruise velocity (rotations per second before gearing reduction). Placed here so both
        // TalonFX and TalonFXS module implementations can reference the same default.
        public static final double kDefaultMotionMagicCruiseVelocity = 100.0;
        // Window (seconds) used when converting cruise velocity into an acceleration setpoint
        // (previously a hard-coded 0.100 literal). Keeping as seconds for clarity.
        public static final double kMotionMagicAccelWindowSec = 0.100;
    }

    // ==========================================
    // Communications / Phoenix Status Signal Constants
    // ==========================================
    public static final class CommsConstants {
        private CommsConstants() {}

        // Default frequency (Hz) for non-odometry Phoenix status signals such as velocity,
        // applied voltage, and current. Previously scattered literals (50.0) are unified here.
        public static final double kDefaultStatusSignalHz = 50.0;
    }

    public static final class PathPlannerConstants {
        private PathPlannerConstants() {}

        // Default PID constants used by the PathPlanner AutoBuilder's holonomic controllers
        public static final double kTranslationP = 5.0;
        public static final double kTranslationI = 0.0;
        public static final double kTranslationD = 0.0;

        public static final double kRotationP = 5.0;
        public static final double kRotationI = 0.0;
        public static final double kRotationD = 0.0;
    }

    // ==========================================
    // Drive Control (non-drive folder) tuning constants
    // ==========================================
    public static final class DriveControlConstants {
        private DriveControlConstants() {}

        // Joystick deadband applied to linear and rotational inputs
        public static final double kDeadband = 0.1;

        // Angle controller gains and profile limits
        public static final double kAngleKP = 5.0;
        public static final double kAngleKD = 0.4;
        public static final double kAngleMaxVelocity = 8.0; // rad/s
        public static final double kAngleMaxAcceleration = 20.0; // rad/s^2

        // Feedforward characterization parameters
        public static final double kFFStartDelay = 2.0; // seconds
        public static final double kFFRampRate = 0.1; // Volts/sec

        // Wheel radius characterization
        public static final double kWheelRadiusMaxVelocity = 0.25; // rad/sec
        public static final double kWheelRadiusRampRate = 0.05; // rad/sec^2
    }

    // ==========================================
    // Robot safety & thresholds
    // ==========================================
    public static final class RobotSafetyConstants {
        private RobotSafetyConstants() {}

        // Low battery threshold that triggers alerts
        public static final double kLowBatteryVoltage = 11.0;
        // Time (seconds) after which a low battery condition will trigger alerts when disabled
        public static final double kLowBatteryDisabledSecs = 2.0;
    }

    // ==========================================
    // Physics Constants
    // ==========================================
    public static final class PhysicsConstants {
        private PhysicsConstants() {}

        // Standard gravity (m/s^2). Use SI units for physics calculations.
        public static final double kStandardGravity = 9.80665;
    }

    // ==========================================
    // Kicker Constants
    // ==========================================
    public static final class KickerConstants {
        private KickerConstants() {}

        // Physical Constants
        public static final Distance kWheelDiameter = Inches.of(2);
        public static final Mass kWheelMass = Pounds.of(1);
        public static final double kGearReduction = 1.0;

        // Current Limits
        public static final Current kStatorCurrentLimit = Amps.of(60);

        // Feedback (PID) Constants
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Theoretical Feedforward
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        // Telemetry
        public static final String kMotorTelemetry = "KickerMotor";
        public static final String kMechTelemetry = "KickerMech";
    }

    // ==========================================
    // Shooter lookup tables (static data moved from code)
    // Each entry: {distance_m, shooter_speed, trajectory_angle_deg, time_of_flight_s}
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

    public static boolean disableHAL = false;

    public static void disableHAL() {
        disableHAL = true;
    }

    /** Checks whether the correct robot is selected when deploying. */
    public static class CheckDeploy {
        public static void main(String... args) {
            if (robot == RobotType.SIMBOT) {
                System.err.println("Cannot deploy, invalid robot selected: " + robot);
                System.exit(1);
            }
        }
    }

    /** Checks that the default robot is selected and tuning mode is disabled. */
    public static class CheckPullRequest {
        public static void main(String... args) {
            if (robot != RobotType.COMPBOT || tuningMode) {
                System.err.println("Do not merge, non-default constants are configured.");
                System.exit(1);
            }
        }
    }
}
