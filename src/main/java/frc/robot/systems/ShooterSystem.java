package frc.robot.systems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kClearDurationSecs;
import static frc.robot.constants.ShooterConstants.HoodConstants.kTunableHoodAngleDeg;
import static frc.robot.constants.ShooterConstants.KickerConstants.kKickerClearAngularVelocity;
import static frc.robot.constants.ShooterConstants.TurretConstants.kRobotToTurretTransform;
import static frc.robot.constants.ShooterConstants.kRefinementConvergenceEpsilon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.SpindexerSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.geometry.Zones;
import frc.robot.util.shooter.HybridTurretUtil;
import frc.robot.util.shooter.ShooterModel;
import frc.robot.util.shooter.ShootingLookupTable;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Aggregated shooter system (high-level grouping of shooter subsystems).
 *
 * <p>This class owns references to the flywheel, kicker, and spindexer subsystems and provides
 * higher-level commands that coordinate them (e.g. shooting routines).
 */
public class ShooterSystem {
    // region Hardware & signals

    public final FlywheelSubsystem flywheel;
    public final KickerSubsystem kicker;
    public final SpindexerSubsystem spindexer;
    public final TurretSubsystem turret;
    public final HoodSubsystem hood;

    // endregion

    // region Initialization helpers

    public ShooterSystem(
            FlywheelSubsystem flywheel,
            KickerSubsystem kicker,
            SpindexerSubsystem spindexer,
            TurretSubsystem turret,
            HoodSubsystem hood) {
        this.flywheel = flywheel;
        this.kicker = kicker;
        this.spindexer = spindexer;
        this.turret = turret;
        this.hood = hood;
    }

    // endregion

    // region Public API (queries & commands)

    private Command clearThenFeed() {
        // Run a short reverse clear, then feed. Do not wait for flywheel spin-up here.
        return clearKicker()
                .andThen(spindexer.feedShooter().alongWith(kicker.feedShooter()))
                .withName("ClearThenFeed");
    }

    /**
     * Clear the shooter: run the kicker and spindexer briefly in reverse to remove jams.
     *
     * @return a command that executes the clear routine
     */
    public Command clearKicker() {
        // Brief closed-loop reverse motion to clear jams, then stop.
        return kicker
                .setVelocity(kKickerClearAngularVelocity)
                .withTimeout(kClearDurationSecs)
                .andThen(kicker.stopNow())
                .withName("ClearKicker");
    }

    /**
     * Test-mode shoot: use the dashboard tunable for hood angle and a tuned flywheel velocity. Does a
     * short clear then feeds. The hood angle is read from the LoggedTunableNumber at runtime via a
     * Supplier so dashboard edits take effect immediately while this command is active.
     */
    public Command testShoot() {
        var hoodRun = hood.setAngle(() -> Degrees.of(kTunableHoodAngleDeg.get())).withTimeout(5);

        var sequence = hoodRun.andThen(flywheel.runToTunedVelocity().alongWith(clearThenFeed()));

        return Commands.parallel(sequence).withName("TestShoot");
    }

    /**
     * Real shoot: automatically choose HUB vs PASS mode based on the robot pose (zones) and aim
     * accordingly. Builds a target supplier from field constants and delegates to {@link
     * #aimAndShoot(Supplier, Supplier, Supplier, int, ShootingLookupTable.Mode)}.
     */
    public Command realShoot(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldSpeeds) {
        // Determine whether we are in a trench/pass-like zone. Treat trench,
        // trench-duck, and
        // bump collections as PASS mode; otherwise use HUB.
        boolean inTrench =
                Zones.TRENCH_ZONES.contains(robotPose).getAsBoolean()
                        || Zones.TRENCH_DUCK_ZONES.contains(robotPose).getAsBoolean()
                        || Zones.BUMP_ZONES.contains(robotPose).getAsBoolean();

        var tableMode = inTrench ? ShootingLookupTable.Mode.PASS : ShootingLookupTable.Mode.HUB;

        // Target supplier: hub center for HUB mode; nearest trench opening for PASS
        // mode.
        Supplier<Translation3d> targetSupplier =
                () -> {
                    if (!inTrench) return AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);
                    var pose = robotPose.get();
                    var left = FieldConstants.LeftTrench.openingTopLeft;
                    var right = FieldConstants.RightTrench.openingTopLeft;
                    double robotY = pose.getTranslation().getY();
                    return Math.abs(robotY - left.getY()) < Math.abs(robotY - right.getY())
                            ? AllianceFlipUtil.apply(left)
                            : AllianceFlipUtil.apply(right);
                };

        return aimAndShoot(robotPose, fieldSpeeds, targetSupplier, 3, tableMode).withName("RealShoot");
    }

    /**
     * Dynamically aim the turret and hood, spin the flywheel, and run the feed pipeline.
     *
     * <p>The returned command continuously tracks a motion-compensated {@code ShotSolution} computed
     * from the provided pose, chassis speeds, and target supplier. It moves the turret and hood,
     * updates flywheel setpoints, runs the feed sequence, and publishes telemetry while active.
     *
     * @param robotPose supplier of the robot pose
     * @param fieldSpeeds supplier of chassis speeds (for lead compensation)
     * @param targetSupplier supplier of the 3D target point
     * @param refinementIterations number of solver refinement iterations for the shot solution
     * @param tableMode lookup table mode (HUB or PASS)
     * @return a command that aims and executes a shot when scheduled
     */
    public Command aimAndShoot(
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> fieldSpeeds,
            Supplier<Translation3d> targetSupplier,
            int refinementIterations,
            ShootingLookupTable.Mode tableMode) {
        // Build a lookup table for the requested mode
        var table = makeLookupTable(tableMode);

        // Live solution supplier (motion-compensated shot solution)
        var solutionSupplier =
                makeSolutionSupplier(robotPose, fieldSpeeds, targetSupplier, refinementIterations, table);

        // Commands to track dynamic targets for turret and hood.
        // Track turret azimuth dynamically from the computed shot solution rather
        // than using a fixed placeholder angle.
        var turretCmd = turret.setAngle(Degrees.of(0));
        // () -> solutionSupplier.get().turretAzimuth()); // .plus(Degrees.of(167)));
        var hoodCmd = hood.setAngle(() -> solutionSupplier.get().hoodAngle());

        // Flywheel follow, feed sequence, and telemetry publisher composed from
        // helpers.
        var flywheelFollow = flywheel.setVelocity(makeFlywheelModelSupplier(solutionSupplier));
        var feedSeq = makeFeedSequence(solutionSupplier);
        var telemetryCmd = makeTelemetryCmd(robotPose, solutionSupplier);
        return Commands.parallel(turretCmd.alongWith(hoodCmd), flywheelFollow, feedSeq, telemetryCmd);
    }

    /**
     * Dynamically aim the turret and hood, spin the flywheel, and run the feed pipeline.
     *
     * <p>The returned command continuously tracks a motion-compensated {@code ShotSolution} computed
     * from the provided pose, chassis speeds, and target supplier. It moves the turret and hood,
     * updates flywheel setpoints, runs the feed sequence, and publishes telemetry while active.
     *
     * @param robotPose supplier of the robot pose
     * @param fieldSpeeds supplier of chassis speeds (for lead compensation)
     * @param targetSupplier supplier of the 3D target point
     * @param refinementIterations number of solver refinement iterations for the shot solution
     * @param tableMode lookup table mode (HUB or PASS)
     * @return a command that aims and executes a shot when scheduled
     */
    public Command aim(
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> fieldSpeeds,
            Supplier<Translation3d> targetSupplier,
            int refinementIterations,
            ShootingLookupTable.Mode tableMode) {
        // Build a lookup table for the requested mode
        var table = makeLookupTable(tableMode);

        // Live solution supplier (motion-compensated shot solution)
        var solutionSupplier =
                makeSolutionSupplier(robotPose, fieldSpeeds, targetSupplier, refinementIterations, table);

        // Return a command that dynamically tracks the computed turret azimuth.
        var turretCmd = turret.setAngle(() -> solutionSupplier.get().turretAzimuth());

        return turretCmd.withName("Aim");
    }

    /**
     * Aim, spin up the flywheel, and feed for a trench/pass shot.
     *
     * <p>Automatically selects the nearest trench opening (left or right) from the robot's current Y
     * position and uses the PASS lookup table. Feeding is NOT gated on the hub shift — the robot will
     * continue feeding as long as the trigger is held, regardless of shift state.
     *
     * @param robotPose supplier of the robot pose
     * @param fieldSpeeds supplier of chassis speeds (for lead compensation)
     * @param refinementIterations number of solver refinement iterations
     * @return a command that aims and feeds a trench shot while scheduled
     */
    public Command aimAndShootTrench(
            Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldSpeeds, int refinementIterations) {
        var table = makeLookupTable(ShootingLookupTable.Mode.PASS);

        // Select the nearest trench opening by robot Y position each loop.
        Supplier<Translation3d> targetSupplier =
                () -> {
                    var left = FieldConstants.LeftTrench.openingTopLeft;
                    var right = FieldConstants.RightTrench.openingTopLeft;
                    double robotY = robotPose.get().getY();
                    return Math.abs(robotY - left.getY()) < Math.abs(robotY - right.getY())
                            ? AllianceFlipUtil.apply(left)
                            : AllianceFlipUtil.apply(right);
                };

        var solutionSupplier =
                makeSolutionSupplier(robotPose, fieldSpeeds, targetSupplier, refinementIterations, table);

        // Turret actively tracks the trench target while trigger is held — overrides the
        // default command tracking for the duration of the trench shot.
        var turretCmd = turret.setAngle(() -> solutionSupplier.get().turretAzimuth());
        var hoodCmd = hood.setAngle(() -> solutionSupplier.get().hoodAngle());
        var flywheelFollow = flywheel.setVelocity(makeFlywheelModelSupplier(solutionSupplier));
        var feedSeq = makeFeedSequenceUngated(solutionSupplier);
        var telemetryCmd = makeTelemetryCmd(robotPose, solutionSupplier);
        return Commands.parallel(turretCmd.alongWith(hoodCmd), flywheelFollow, feedSeq, telemetryCmd)
                .withName("TrenchShoot");
    }

    // Small helper factories to keep the main flow concise and easier to read.
    private ShootingLookupTable makeLookupTable(ShootingLookupTable.Mode mode) {
        return new ShootingLookupTable(mode);
    }

    private Supplier<HybridTurretUtil.ShotSolution> makeSolutionSupplier(
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> fieldSpeeds,
            Supplier<Translation3d> targetSupplier,
            int refinementIterations,
            ShootingLookupTable table) {
        return () ->
                HybridTurretUtil.computeMovingShot(
                        robotPose.get(),
                        fieldSpeeds.get(),
                        targetSupplier.get(),
                        refinementIterations,
                        kRefinementConvergenceEpsilon,
                        table);
    }

    private Supplier<AngularVelocity> makeFlywheelModelSupplier(
            Supplier<HybridTurretUtil.ShotSolution> solutionSupplier) {
        return () -> ShooterModel.flywheelSpeedForDistance(solutionSupplier.get().leadDistance());
    }

    private Command makeFeedSequence(Supplier<HybridTurretUtil.ShotSolution> solutionSupplier) {
        // Feed only while the shifted shift is active — stops automatically at shift
        // boundary without requiring driver input. Flywheel keeps spinning in parallel.
        return Commands.sequence(
                        clearKicker(),
                        spindexer
                                .feedShooter()
                                .alongWith(kicker.feedShooter())
                                .onlyWhile(() -> HubShiftUtil.getShiftedShiftInfo().active()))
                .withName("FeedSequence");
    }

    private Command makeFeedSequenceUngated(
            Supplier<HybridTurretUtil.ShotSolution> solutionSupplier) {
        // Feed freely without shift-gating — used for trench/pass shots where there
        // is no hub shift boundary to respect. Feeds until the command is cancelled
        // (i.e. while the trigger is held).
        return Commands.sequence(clearKicker(), spindexer.feedShooter().alongWith(kicker.feedShooter()))
                .withName("FeedSequenceUngated");
    }

    /**
     * Compute the turret origin (world XY) from the robot pose using the configured robot->turret
     * transform. This avoids allocations and keeps the telemetry math tidy.
     */
    private static Translation2d turretOrigin(Pose2d rp) {
        double theta = rp.getRotation().getRadians();
        double ox = kRobotToTurretTransform.getTranslation().getX();
        double oy = kRobotToTurretTransform.getTranslation().getY();
        double turretX = rp.getX() + ox * Math.cos(theta) - oy * Math.sin(theta);
        double turretY = rp.getY() + ox * Math.sin(theta) + oy * Math.cos(theta);
        return new Translation2d(turretX, turretY);
    }

    private Command makeTelemetryCmd(
            Supplier<Pose2d> robotPose, Supplier<HybridTurretUtil.ShotSolution> solutionSupplier) {
        return Commands.run(
                        () -> {
                            var sol = solutionSupplier.get();

                            // Lead distance (includes motion-predicted lead)
                            Logger.recordOutput(
                                    "ShooterTelemetry/leadDistanceMeters", sol.leadDistance().in(Meters));

                            // Distance from turret origin to alliance hub center
                            var hub = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);
                            var turretXY = turretOrigin(robotPose.get());
                            double hubDx = hub.toTranslation2d().getX() - turretXY.getX();
                            double hubDy = hub.toTranslation2d().getY() - turretXY.getY();
                            Logger.recordOutput("ShooterTelemetry/hubDistanceMeters", Math.hypot(hubDx, hubDy));

                            var model = ShooterModel.flywheelSpeedForDistance(sol.leadDistance());
                            Logger.recordOutput("ShooterTelemetry/lutFlywheelRPM", sol.flywheelSpeed().in(RPM));
                            Logger.recordOutput("ShooterTelemetry/modelFlywheelRPM", model.in(RPM));
                            Logger.recordOutput(
                                    "ShooterTelemetry/deltaRPM", model.in(RPM) - sol.flywheelSpeed().in(RPM));
                            Logger.recordOutput("ShooterTelemetry/lutHoodDegrees", sol.hoodAngle().in(Degrees));
                            Logger.recordOutput("ShooterTelemetry/lutToFSeconds", sol.timeOfFlight().in(Seconds));
                            Logger.recordOutput("ShooterTelemetry/isValid", sol.isValid());
                        })
                .withName("ShooterTelemetryPublisher");
    }

    public Command clearShooterSystem() {
        return flywheel
                .clearFlywheel()
                .alongWith(kicker.clearKicker().alongWith(spindexer.clearSpindexer()));
    }

    // endregion

    // region Private helpers

    /**
     * Interrupt any active shooting commands. This cancels pipelines but does not command zero
     * setpoints (allowing flywheel/spindexer to coast). Use {@code stopNow()} / {@code stopHold()} on
     * subsystems when an explicit stop is required.
     *
     * @return a Command that cancels shooting activity and brings mechanisms to a safe idle
     */
    public Command interruptShooting() {
        return Commands.runOnce(() -> {}, flywheel, kicker, spindexer, turret, hood)
                .withName("InterruptShooting");
    }

    public Command stopShooting() {
        return flywheel.stopNow().alongWith(kicker.stopNow()).alongWith(spindexer.stopNow());
    }

    // endregion
}
