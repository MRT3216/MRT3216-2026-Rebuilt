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
    /** Selected robot hardware profile used at runtime (COMPBOT or SIMBOT). */
    public static final RobotType robot = RobotType.COMPBOT;

    /** Whether the project is running in tuning mode (enables extra telemetry/tunables). */
    public static final boolean tuningMode = false;

    /** Main control loop period (seconds). */
    public static final double loopPeriodSecs = 0.02;

    /** Watchdog period for loop monitoring (seconds). */
    public static final double loopPeriodWatchdogSecs = 0.2;

    /** Currently active runtime mode (REAL, SIM, REPLAY). */
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    /** Operation modes for the robot (simulation, real robot, etc.). */
    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    /** Robot hardware profile selection used to pick IO implementations. */
    public enum RobotType {
        /** Default competition robot hardware. */
        COMPBOT,
        /** Lightweight simulator-only configuration. */
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
        // Physical Constants
        /** Diameter of the shooter wheel. */
        public static final Distance kWheelDiameter = Inches.of(3);

        /** Mass of the shooter wheel. */
        public static final Mass kWheelMass = Pounds.of(3);

        /** Overall gear reduction from motor to wheel (ratio). */
        public static final double kGearReduction = 1.0;

        // Current Limits
        /** Stator current limit applied to shooter motors. */
        public static final Current kStatorCurrentLimit = Amps.of(80);

        // Feedback (PID) Constants
        /** Proportional gain for the shooter closed-loop controller. */
        public static final double kP = 1.0;

        /** Integral gain for the shooter closed-loop controller. */
        public static final double kI = 0.0;

        /** Derivative gain for the shooter closed-loop controller. */
        public static final double kD = 0.0;

        // Theoretical Feedforward (Kraken X60 FOC)
        /** Static/feedforward voltage to break friction (Volts). */
        public static final double kS = 0.15; // Volts to break friction

        /** Velocity feedforward constant (Volts per RPM). */
        public static final double kV = 0.00207; // Volts per RPM

        /** Acceleration feedforward constant (Volts per RPM^2). */
        public static final double kA = 0.0001; // Volts per RPM^2

        /** Telemetry update frequency (Hz) for Phoenix status signals related to the shooter. */
        public static final double kUpdateHz = 50.0;
        public static final String kMotorTelemetry = "FlywheelMotor";
        public static final String kMechTelemetry = "FlywheelMech";
    }

    // endregion

    // region Turret Constants
    public static final class TurretConstants {
        private TurretConstants() {}

        // Hardware & Motor Configuration

        /** Effective gearing ratio for the turret mechanism. */
        public static final double kGearing = 32.4000;

        /** Whether the turret motor output is inverted. */
        public static final boolean kMotorInverted = false;

        /** Stator current limit for the turret motor. */
        public static final Current kStatorCurrentLimit = Amps.of(60);

        /** Moment of inertia of the turret assembly (kg·m^2). */
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0502269403);

        // PID / Motion Profiling (Trapezoidal Limits)
        /** Proportional gain for turret position control. */
        public static final double kP = 10.0;

        /** Integral gain for turret position control. */
        public static final double kI = 0.0;

        /** Derivative gain for turret position control. */
        public static final double kD = 2.0;

        /** Maximum allowed turret velocity (degrees per second). */
        public static final double kMaxVelocityDegPerSec = 90.0;

        /** Maximum allowed turret acceleration (degrees per second squared). */
        public static final double kMaxAccelDegPerSec2 = 45.0;

        // Feedforward Constants
        /** Feedforward static term (Volts) for turret motion. */
        public static final double kS = 0.1; // Static friction voltage

        /** Feedforward velocity term (Volts per unit velocity). */
        public static final double kV = 0.12; // Velocity feedforward

        /** Feedforward acceleration term (Volts per unit acceleration). */
        public static final double kA = 0.01; // Acceleration feedforward

        // Geometry & Transforms
        // Physical offset from robot origin (center-floor) to turret pivot point
        /** X offset (meters) from robot origin to turret pivot. */
        public static final Distance kTurretOffsetX = Inches.of(0.0);

        /** Y offset (meters) from robot origin to turret pivot. */
        public static final Distance kTurretOffsetY = Inches.of(0.0);

        /** Z offset (meters) from floor to turret pivot height. */
        public static final Distance kTurretOffsetZ = Inches.of(18.5);
        public static final Transform3d kRobotToTurretTransform =
                new Transform3d(
                        new Translation3d(
                                kTurretOffsetX.in(Meters), kTurretOffsetY.in(Meters), kTurretOffsetZ.in(Meters)),
                        new Rotation3d());

        // Mechanism Limits
        /** Hard physical limit maximum angle for the turret. */
        public static final Angle kHardLimitMax = Degrees.of(180);

        /** Hard physical limit minimum angle for the turret. */
        public static final Angle kHardLimitMin = Degrees.of(-180);

        /** Soft (software) limit maximum angle for normal operation. */
        public static final Angle kSoftLimitMax = Degrees.of(170);

        /** Soft (software) limit minimum angle for normal operation. */
        public static final Angle kSoftLimitMin = Degrees.of(-170);

        /** Default starting turret position at boot. */
        public static final Angle kStartingPosition = Degrees.of(0);

        // Validation Ranges
        /** Minimum validated shooting distance (meters). */
        public static final Distance kMinShootingDistance = Meters.of(1.5);

        /** Maximum validated shooting distance (meters). */
        public static final Distance kMaxShootingDistance = Meters.of(12.0);

        // Physics & Ramp Constants (TurretCalculator)
        // v = BaseVel + (Multiplier * distance ^ Power)
        /** Base launch velocity used by TurretCalculator heuristics. */
        public static final LinearVelocity kBaseVel = InchesPerSecond.of(300);

        /** Multiplier applied to distance when computing heuristic velocity. */
        public static final double kVelMultiplier = 0.5;

        /** Exponent applied to distance when computing heuristic velocity. */
        public static final double kVelPower = 1.2;

        // Clearance for Funnel Obstacle
        /** Additional clearance distance above funnel used for special-case shots. */
        public static final Distance kDistanceAboveFunnel = Inches.of(12.0);

        /** Funnel radius (inches) used for clearance calculations. */
        public static final Distance kFunnelRadius = Inches.of(24.0);

        /** Funnel height used for clearance and trajectory planning. */
        public static final Distance kFunnelHeight = Inches.of(104.0);
        public static final String kMotorTelemetry = "TurretMotor";
        public static final String kMechTelemetry = "TurretMech";
    }

    // ==========================================
    // Drive / PathPlanner Constants
    // ==========================================
    public static final class DriveConstants {
        private DriveConstants() {}

        // Robot physical properties used by PathPlanner
        /** Robot mass in kilograms. */
        public static final double kRobotMassKg = 74.088;

        /** Robot moment of inertia used for path planning. */
        public static final double kRobotMOI = 6.883;

        /** Wheel coefficient for drivetrain modeling and tuning. */
        public static final double kWheelCoef = 1.2;

        // Telemetry / tuning
        /** Odometry update frequency (Hz) for networked field devices. */
        public static final double kOdometryFreqNetworkFD = 250.0;

        /** Odometry update frequency (Hz) when obtained via CAN devices. */
        public static final double kOdometryFreqCAN = 100.0;

        // MotionMagic / low-level motion defaults used by TalonFX-based module implementations
        // Cruise velocity (rotations per second before gearing reduction). Placed here so both
        // TalonFX and TalonFXS module implementations can reference the same default.
        /** Default cruise velocity used by MotionMagic-style controllers. */
        public static final double kDefaultMotionMagicCruiseVelocity = 100.0;

        /** Window (seconds) used when converting cruise velocity into an acceleration setpoint. */
        public static final double kMotionMagicAccelWindowSec = 0.100;
    }

    // ==========================================
    // Communications / Phoenix Status Signal Constants
    // ==========================================
    public static final class CommsConstants {
        private CommsConstants() {}

        /**
         * Default frequency (Hz) for non-odometry Phoenix status signals such as velocity, applied
         * voltage, and current. Previously scattered literals (50.0) are unified here.
         */
        public static final double kDefaultStatusSignalHz = 50.0;
    }

    public static final class PathPlannerConstants {
        private PathPlannerConstants() {}

        // Default PID constants used by the PathPlanner AutoBuilder's holonomic controllers
        /** Translation controller proportional gain. */
        public static final double kTranslationP = 5.0;

        /** Translation controller integral gain. */
        public static final double kTranslationI = 0.0;

        /** Translation controller derivative gain. */
        public static final double kTranslationD = 0.0;

        /** Rotation controller proportional gain. */
        public static final double kRotationP = 5.0;

        /** Rotation controller integral gain. */
        public static final double kRotationI = 0.0;

        /** Rotation controller derivative gain. */
        public static final double kRotationD = 0.0;
    }

    // ==========================================
    // Drive Control (non-drive folder) tuning constants
    // ==========================================
    public static final class DriveControlConstants {
        private DriveControlConstants() {}
        // Joystick deadband applied to linear and rotational inputs
        /** Deadband applied to joystick axes to ignore small noise. */
        public static final double kDeadband = 0.1;

        // Angle controller gains and profile limits
        /** Angle controller proportional gain. */
        public static final double kAngleKP = 5.0;

        /** Angle controller derivative gain. */
        public static final double kAngleKD = 0.4;

        /** Angle controller maximum velocity (rad/s). */
        public static final double kAngleMaxVelocity = 8.0; // rad/s

        /** Angle controller maximum acceleration (rad/s^2). */
        public static final double kAngleMaxAcceleration = 20.0; // rad/s^2

        // Feedforward characterization parameters
        /** Seconds to wait before starting feedforward characterization. */
        public static final double kFFStartDelay = 2.0; // seconds

        /** Ramp rate for feedforward characterization (Volts/sec). */
        public static final double kFFRampRate = 0.1; // Volts/sec

        // Wheel radius characterization
        /** Max velocity used during wheel radius characterization. */
        public static final double kWheelRadiusMaxVelocity = 0.25; // rad/sec

        /** Ramp rate used during wheel radius characterization. */
        public static final double kWheelRadiusRampRate = 0.05; // rad/sec^2
    }

    // ==========================================
    // Robot safety & thresholds
    // ==========================================
    public static final class RobotSafetyConstants {
        private RobotSafetyConstants() {}
        // Low battery threshold that triggers alerts
        /** Low battery voltage threshold that will trigger alerts. */
        public static final double kLowBatteryVoltage = 11.0;

        /** Time (seconds) after which a low battery condition will trigger alerts when disabled. */
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
        /** Diameter of the kicker wheel. */
        public static final Distance kWheelDiameter = Inches.of(2);

        /** Mass of the kicker wheel. */
        public static final Mass kWheelMass = Pounds.of(1);

        /** Gear reduction ratio for the kicker mechanism. */
        public static final double kGearReduction = 1.0;

        // Current Limits
        /** Stator current limit applied to the kicker motor. */
        public static final Current kStatorCurrentLimit = Amps.of(60);

        // Feedback (PID) Constants
        /** Proportional gain for the kicker closed-loop controller. */
        public static final double kP = 1.0;

        /** Integral gain for the kicker closed-loop controller. */
        public static final double kI = 0.0;

        /** Derivative gain for the kicker closed-loop controller. */
        public static final double kD = 0.0;

        // Theoretical Feedforward
        /** Static/feedforward voltage term for the kicker. */
        public static final double kS = 0.0;

        /** Velocity feedforward term for the kicker. */
        public static final double kV = 0.0;

        /** Acceleration feedforward term for the kicker. */
        public static final double kA = 0.0;

        // Telemetry
        /** Telemetry label used for the kicker motor. */
        public static final String kMotorTelemetry = "KickerMotor";

        /** Telemetry label used for the kicker mechanism. */
        public static final String kMechTelemetry = "KickerMech";
    }

    // ==========================================
    // Spindexer Constants
    // ==========================================
    public static final class SpindexerConstants {
        private SpindexerConstants() {}

        // Effective gear reduction from motor to roller (motor rotations per roller rotation)
        public static final double kGearing = 5.0;

        // Motor inversion
        public static final boolean kMotorInverted = false;

        // Stator current limit
        public static final Current kStatorCurrentLimit = Amps.of(30);

        // PID gains for closed-loop velocity control (tune as needed)
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward terms (Volts units) - tune via characterization
        public static final double kS = 0.05;
        public static final double kV = 0.01;
        public static final double kA = 0.0;

        // Telemetry labels
        public static final String kMotorTelemetry = "SpindexerMotor";
        public static final String kMechTelemetry = "SpindexerMech";
    }

    // ==========================================
    // Shooter lookup tables (static data moved from code)
    // Each entry: {distance_m, shooter_speed, trajectory_angle_deg, time_of_flight_s}
    /**
     * Static arrays containing shooting lookup datasets. Each row contains a tuple of {distance_m,
     * shooter_speed, trajectory_angle_deg, time_of_flight_s}. These are used by {@link
     * frc.robot.util.ShootingLookupTable} to build interpolation tables for shooter setpoints.
     */
    public static final class ShooterLookupTables {
        private ShooterLookupTables() {}

        /**
         * Static shooting lookup datasets. Each entry is a 4-tuple: {distance_m, shooter_speed,
         * trajectory_angle_deg, time_of_flight_s}. These tables are used by {@link
         * frc.robot.util.ShootingLookupTable}.
         */
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
