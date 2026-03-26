// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants.DriveControlConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterConstants.HybridAimingConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
    private static final double DEADBAND = DriveControlConstants.kDeadband;
    private static final double ANGLE_KP = DriveControlConstants.kAngleKP;
    private static final double ANGLE_KD = DriveControlConstants.kAngleKD;
    private static final double ANGLE_MAX_VELOCITY = DriveControlConstants.kAngleMaxVelocity;
    private static final double ANGLE_MAX_ACCELERATION = DriveControlConstants.kAngleMaxAcceleration;
    private static final double FF_START_DELAY = DriveControlConstants.kFFStartDelay; // Secs
    private static final double FF_RAMP_RATE = DriveControlConstants.kFFRampRate; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY =
            DriveControlConstants.kWheelRadiusMaxVelocity; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE =
            DriveControlConstants.kWheelRadiusRampRate; // Rad/Sec^2

    // Hybrid aiming constants
    private static final double HYBRID_HOME_ANGLE_DEG = HybridAimingConstants.kTurretHomeAngleDeg;
    private static final double HYBRID_DEADBAND_DEG = HybridAimingConstants.kTurretDeadbandDeg;
    private static final double HYBRID_MARGIN_DEG = HybridAimingConstants.kThresholdMarginDeg;
    private static final double HYBRID_HEADING_KP = HybridAimingConstants.kHeadingKP;
    private static final double HYBRID_HEADING_KD = HybridAimingConstants.kHeadingKD;
    private static final double HYBRID_HEADING_MAX_VEL = HybridAimingConstants.kHeadingMaxVelocity;
    private static final double HYBRID_HEADING_MAX_ACCEL =
            HybridAimingConstants.kHeadingMaxAcceleration;

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(Translation2d.kZero, linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
                .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity =
                            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds =
                            new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega * drive.getMaxAngularSpeedRadPerSec());
                    boolean isFlipped =
                            DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                },
                drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier) {

        // Create PID controller
        ProfiledPIDController angleController =
                new ProfiledPIDController(
                        ANGLE_KP,
                        0.0,
                        ANGLE_KD,
                        new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            double omega =
                                    angleController.calculate(
                                            drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds =
                                    new ChassisSpeeds(
                                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                            omega);
                            boolean isFlipped =
                                    DriverStation.getAlliance().isPresent()
                                            && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            speeds,
                                            isFlipped
                                                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                                    : drive.getRotation()));
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        }),

                // Allow modules to orient
                Commands.run(
                                () -> {
                                    drive.runCharacterization(0.0);
                                },
                                drive)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * FF_RAMP_RATE;
                                    drive.runCharacterization(voltage);
                                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                drive)

                        // When cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Drive FF Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(
                                () -> {
                                    limiter.reset(0.0);
                                }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(
                                () -> {
                                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRotation();
                                    state.gyroDelta = 0.0;
                                }),

                        // Update gyro delta
                        Commands.run(
                                        () -> {
                                            var rotation = drive.getRotation();
                                            state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                                            state.lastAngle = rotation;
                                        })

                                // When cancelled, calculate and print results
                                .finallyDo(
                                        () -> {
                                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                            }
                                            double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                                            NumberFormat formatter = new DecimalFormat("#0.000");
                                            System.out.println(
                                                    "********** Wheel Radius Characterization Results **********");
                                            System.out.println(
                                                    "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                            System.out.println(
                                                    "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                            System.out.println(
                                                    "\tWheel Radius: "
                                                            + formatter.format(wheelRadius)
                                                            + " meters, "
                                                            + formatter.format(Units.metersToInches(wheelRadius))
                                                            + " inches");
                                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }

    public static Rotation2d snap45(Rotation2d rotation) {
        double currentAngle = rotation.getDegrees();
        double snappedAngle = Math.round(currentAngle / 45.0) * 45.0;
        return Rotation2d.fromDegrees(snappedAngle);
    }

    // =========================================================================
    // Hybrid Aiming — drivetrain coarse heading + turret fine correction
    // =========================================================================

    /**
     * Field-relative drive with automatic heading correction toward a target point.
     *
     * <p><b>This is the drivetrain half of the hybrid aiming system.</b> The driver retains full
     * translational control (left stick) and rotational control (right stick). When the
     * robot-relative angle to the target grows large enough, a profiled PID heading controller blends
     * additional rotational velocity to steer the chassis toward the target.
     *
     * <p>Three zones control the heading correction strength (measured from turret home):
     *
     * <ul>
     *   <li><b>Inner zone</b> (0° – inner threshold): turret handles aiming alone, heading PID is
     *       reset.
     *   <li><b>Ramp zone</b> (inner threshold – deadband): heading PID output scales linearly from 0%
     *       → 100%, giving the chassis a smooth head start.
     *   <li><b>Outer zone</b> (&gt; deadband): full heading PID correction.
     * </ul>
     *
     * <p>All tuning constants (PID gains, deadband, margin, home angle) are read from {@link
     * HybridAimingConstants}.
     *
     * @param drive the swerve drive subsystem
     * @param xSupplier left stick Y axis (forward/back) — negated by caller
     * @param ySupplier left stick X axis (left/right) — negated by caller
     * @param omegaSupplier right stick X axis (rotation) — negated by caller
     * @param targetSupplier field-relative 2D target point (e.g. hub center, alliance-flipped)
     * @param robotPoseSupplier current robot field pose
     * @param aimEnabled supplier that gates whether the heading controller is active. When false
     *     (e.g. driver not holding shoot trigger), this behaves exactly like {@link
     *     #joystickDrive(Drive, DoubleSupplier, DoubleSupplier, DoubleSupplier)}.
     * @return a command that drives with optional heading aim assist
     */
    public static Command joystickDriveAimAtTarget(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            Supplier<Translation2d> targetSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<Boolean> aimEnabled) {

        // Heading PID — only active when the target is outside the turret deadband.
        ProfiledPIDController headingController =
                new ProfiledPIDController(
                        HYBRID_HEADING_KP,
                        0.0,
                        HYBRID_HEADING_KD,
                        new TrapezoidProfile.Constraints(HYBRID_HEADING_MAX_VEL, HYBRID_HEADING_MAX_ACCEL));
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        double deadbandRad = Math.toRadians(HYBRID_DEADBAND_DEG);
        double homeAngleRad = Math.toRadians(HYBRID_HOME_ANGLE_DEG);
        double marginRad = Math.toRadians(HYBRID_MARGIN_DEG);
        double innerThresholdRad = deadbandRad - marginRad;

        return Commands.run(
                        () -> {
                            // --- Translation (identical to joystickDrive) ---
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // --- Rotation: driver + optional heading assist ---
                            double driverOmega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
                            driverOmega = Math.copySign(driverOmega * driverOmega, driverOmega);

                            double headingOmega = 0.0;
                            boolean isAiming = aimEnabled.get();

                            if (isAiming) {
                                // Compute robot-relative angle to target
                                Pose2d pose = robotPoseSupplier.get();
                                Translation2d target = targetSupplier.get();
                                double dx = target.getX() - pose.getX();
                                double dy = target.getY() - pose.getY();
                                double fieldAngleToTarget = Math.atan2(dy, dx);

                                // Robot-relative angle to target, measured from the turret home
                                // direction. If home=0° this is the angle from robot front; if
                                // home=180° this is the angle from robot rear.
                                double robotRelativeAngle =
                                        MathUtil.angleModulus(
                                                fieldAngleToTarget - pose.getRotation().getRadians() - homeAngleRad);

                                Logger.recordOutput(
                                        "HybridAiming/robotRelativeAngleDeg", Math.toDegrees(robotRelativeAngle));
                                Logger.recordOutput(
                                        "HybridAiming/outsideDeadband", Math.abs(robotRelativeAngle) > deadbandRad);

                                double absAngle = Math.abs(robotRelativeAngle);
                                double rampFactor; // 0.0 = turret only, 1.0 = full drivetrain assist

                                if (absAngle <= innerThresholdRad) {
                                    // === Inner zone: turret handles it alone ===
                                    rampFactor = 0.0;
                                    headingController.reset(pose.getRotation().getRadians());
                                } else if (absAngle >= deadbandRad) {
                                    // === Outer zone: full drivetrain correction ===
                                    rampFactor = 1.0;
                                } else {
                                    // === Ramp zone: linear interpolation from 0% → 100% ===
                                    rampFactor = (absAngle - innerThresholdRad) / marginRad;
                                }

                                if (rampFactor > 0.0) {
                                    // Setpoint = field angle to target minus the home offset, so the
                                    // turret-home face of the robot points at the target.
                                    double desiredHeading = MathUtil.angleModulus(fieldAngleToTarget - homeAngleRad);
                                    headingOmega =
                                            rampFactor
                                                    * headingController.calculate(
                                                            pose.getRotation().getRadians(), desiredHeading);
                                }

                                Logger.recordOutput("HybridAiming/rampFactor", rampFactor);
                                Logger.recordOutput("HybridAiming/headingOmega", headingOmega);
                            } else {
                                // Aiming disabled — reset controller to avoid stale state.
                                headingController.reset(robotPoseSupplier.get().getRotation().getRadians());
                            }

                            Logger.recordOutput("HybridAiming/aimEnabled", isAiming);

                            // Combine driver input and heading correction.
                            // Driver omega is scaled to max angular speed; heading omega is in
                            // raw rad/s already.
                            double totalOmega = driverOmega * drive.getMaxAngularSpeedRadPerSec() + headingOmega;

                            // Build field-relative chassis speeds
                            ChassisSpeeds speeds =
                                    new ChassisSpeeds(
                                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                            totalOmega);
                            boolean isFlipped =
                                    DriverStation.getAlliance().isPresent()
                                            && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            speeds,
                                            isFlipped
                                                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                                    : drive.getRotation()));
                        },
                        drive)
                .beforeStarting(
                        () -> headingController.reset(robotPoseSupplier.get().getRotation().getRadians()));
    }
}
