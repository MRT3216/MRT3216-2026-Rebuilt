// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;

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

    public static final class TurretConstants {
        private TurretConstants() {}

        // Hardware
        public static final int kPivotMotorId = 53;

        // PID / Motion limits
        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 2.0;
        public static final double kMaxVelocityDegPerSec = 90.0;
        public static final double kMaxAccelDegPerSec2 = 45.0;

        // Motor configuration
        public static final double kGearing = 32.4000;
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(60);

        // Mechanism properties & limits
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0502269403);
        public static final Angle kHardLimitMax = Degrees.of(180);
        public static final Angle kHardLimitMin = Degrees.of(-180);
        public static final Angle kSoftLimitMax = Degrees.of(170);
        public static final Angle kSoftLimitMin = Degrees.of(-170);
        public static final Angle kStartingPosition = Degrees.of(0);

        // Telemetry keys
        public static final String kMotorTelemetry = "TurretMotor";
        public static final String kMechTelemetry = "TurretMech";
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
