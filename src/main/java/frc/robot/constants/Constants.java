// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Central repository of robot constants. Keep this file tidy: avoid duplicate nested classes and
 * prefer Javadoc over short inline comment headings.
 */
public final class Constants {
    // ---------------------------------------------------------------------
    // Global / runtime settings
    // ---------------------------------------------------------------------

    /** Selected robot hardware profile used at runtime (COMPBOT or SIMBOT). */
    public static final RobotType robot = RobotType.COMPBOT;

    /**
     * When true, enable runtime tuning/test bindings even if the Driver Station is not in Test mode.
     * This flag allows enabling tuning behavior via code/config rather than relying on the Driver
     * Station Test mode switch.
     */
    public static boolean tuningMode = true;

    /**
     * Use {@link #getMode()} to determine the runtime mode (REAL, SIM, REPLAY). Extra telemetry and
     * interactive test bindings are enabled at runtime (for example from {@code Robot.testInit()})
     * instead of via a compile-time boolean.
     */

    /** Main control loop period (seconds). */
    public static final double loopPeriodSecs = 0.02;

    /** Watchdog period for loop monitoring (seconds). */
    public static final double loopPeriodWatchdogSecs = 0.2;

    // Provide a Littleton-style runtime Mode getter. This returns REAL/REPLAY/SIM based on the
    // selected `robot` profile and `RobotBase.isReal()` when appropriate.
    public static Mode getMode() {
        return switch (robot) {
            case COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.SIM;
            case SIMBOT -> Mode.SIM;
            default -> RobotBase.isReal() ? Mode.REAL : Mode.SIM;
        };
    }

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

    /**
     * Returns the YAMS telemetry verbosity to use for mechanisms. Centralized so it is easy to change
     * behavior for tuning vs competition in one place.
     *
     * <ul>
     *   <li><b>HIGH</b> — All fields: tunable gains (kP/kI/kD/kS/kV/kA/kG), tunable setpoints, motor
     *       temp, limits, ramp rates, motion-profile params, boolean flags. Best for tuning.
     *   <li><b>MID</b> — Adds output voltage, stator/supply current, rotor position/velocity to LOW.
     *       Good balance for competition debugging.
     *   <li><b>LOW</b> — Setpoint position/velocity, measurement position/velocity, mechanism
     *       position/velocity. Minimal overhead for match play.
     * </ul>
     */
    public static TelemetryVerbosity telemetryVerbosity() {
        if (tuningMode) {
            return TelemetryVerbosity.HIGH;
        }
        // Competition: MID gives voltage + current data for post-match analysis
        // without the overhead of publishing every tunable gain each cycle.
        return TelemetryVerbosity.MID;
    }

    // ---------------------------------------------------------------------
    // Drive & motion-related constants
    // ---------------------------------------------------------------------

    // region Drive

    /** Drive-related physical constants used by the drivetrain and odometry. */
    public static final class DriveConstants {
        private DriveConstants() {}

        public static final double kRobotMassKg = 63.503; // 140 lbs (with bumpers + battery)
        public static final double kRobotMOI = 6.883;

        /**
         * Wheel coefficient of friction (µ) used by PathPlanner's traction model for acceleration
         * limiting. Higher = PathPlanner trusts the robot can accelerate/decelerate harder.
         *
         * <p>Reference team values (2026 season):
         *
         * <ul>
         *   <li>6328 Mechanical Advantage: 1.5
         *   <li>Hammerheads 5000: 2.255 (aggressive — Colson or high-grip tread)
         *   <li>Most teams: 1.0–1.5 (conservative default)
         * </ul>
         *
         * <p>TODO: Measure our actual wheel COF using the slip test described in docs/TuningGuide.md
         * "Step 6: Slip Current Measurement". Update both here and in {@code Drive.java} (WHEEL_COF)
         * and {@code src/main/deploy/pathplanner/settings.json}.
         */
        public static final double kWheelCoef = 1.2;

        public static final double kOdometryFreqNetworkFD = 250.0;
        public static final double kOdometryFreqCAN = 100.0;
        public static final double kDefaultMotionMagicCruiseVelocity = 100.0;
        public static final double kMotionMagicAccelWindowSec = 0.100;
    }

    // endregion

    // region DriveControl

    /**
     * Tunable control gains and limits used by drive controllers (angle/velocity controllers).
     *
     * <p>The FF characterization parameters ({@code kFFStartDelay}, {@code kFFRampRate}) match
     * AdvantageKit's template defaults. Hammerheads 5000 uses similar values (delay=2.0s,
     * ramp=0.1V/s). These are safe defaults for the drive feedforward characterization routine.
     */
    public static final class DriveControlConstants {
        private DriveControlConstants() {}

        public static final double kDeadband = 0.1;
        public static final double kAngleKP = 5.0;
        public static final double kAngleKD = 0.4;
        public static final double kAngleMaxVelocity = 8.0;
        public static final double kAngleMaxAcceleration = 20.0;
        public static final double kFFStartDelay = 2.0;
        public static final double kFFRampRate = 0.1;
        public static final double kWheelRadiusMaxVelocity =
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / TunerConstants.kWheelRadius.in(Meters);
        public static final double kWheelRadiusRampRate = 0.05;
    }

    // endregion

    // region PathPlanner

    /**
     * Path planner tuning gains used by autonomous path-following controllers.
     *
     * <p>Reference team values (2026 season):
     *
     * <table>
     *   <tr><th>Team</th><th>Translation P</th><th>Rotation P</th><th>Notes</th></tr>
     *   <tr><td>LASA PR</td><td>3.0</td><td>4.0</td><td>No I or D terms</td></tr>
     *   <tr><td>Hammerheads 5000</td><td>3.0</td><td>2.0</td><td>I=0.05 on both</td></tr>
     *   <tr><td>BroncBotz 3481</td><td>5.0</td><td>5.0</td><td>Same as us</td></tr>
     *   <tr><td>6328</td><td>8.0 (linear)</td><td>4.0 (theta)</td><td>Uses Choreo, not PP</td></tr>
     * </table>
     *
     * <p>Our values (5.0/5.0) are on the aggressive end. If autonomous paths overshoot or oscillate,
     * try reducing to 3.0. See docs/TuningGuide.md "Step 8: PathPlanner Configuration".
     */
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

    // ---------------------------------------------------------------------
    // Safety / communications / misc infrastructure
    // ---------------------------------------------------------------------

    // region RobotSafety

    /** Robot-wide safety thresholds used by higher-level systems (battery, disable timeouts). */
    public static final class RobotSafetyConstants {
        private RobotSafetyConstants() {}

        public static final double kLowBatteryVoltage = 11.0;
        public static final double kLowBatteryDisabledSecs = 2.0;
    }

    // endregion

    // region LEDs

    /** LED configuration constants (length / effect tuning). */
    public static final class LEDsConstants {
        private LEDsConstants() {}

        // TODO - update to led length
        public static final int kNumLEDs = 60;
    }

    // endregion

    // region Communications

    /** Communication-related constants (status heartbeat frequencies, etc.). */
    public static final class CommsConstants {
        private CommsConstants() {}

        // Default telemetry frequency (Hz) used for pushed Phoenix/status signals.
        // Kept here as the single source of truth to avoid duplication.
        public static final int DEFAULT_TELEMETRY_HZ = 50;
        public static final int HIGH_TELEMETRY_HZ = 500;
    }

    // endregion

    // ---------------------------------------------------------------------
    // Utility / developer helpers
    // ---------------------------------------------------------------------

    // region Utility

    /** Small developer helpers and CLI checks used during deploys and PR gating. */
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
            if (robot != RobotType.COMPBOT) {
                System.err.println("Do not merge, non-default constants are configured.");
                System.exit(1);
            }
        }
    }

    // endregion

}
