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

    // region Public commands

    /**
     * Clear the shooter: run the kicker briefly in reverse to dislodge jams.
     *
     * @return a command that executes the clear routine
     */
    public Command clearKicker() {
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
        var feedAfterClear =
                clearKicker().andThen(spindexer.feedShooter().alongWith(kicker.feedShooter()));
        return hoodRun
                .andThen(flywheel.runToTunedVelocity().alongWith(feedAfterClear))
                .withName("TestShoot");
    }

    /**
     * Automatically choose HUB vs PASS mode based on the robot pose (zones) and aim accordingly. Zone
     * evaluation happens each loop — the mode may switch if the robot crosses a zone boundary while
     * the command is running.
     */
    public Command realShoot(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldSpeeds) {
        // Evaluate zone membership per-loop so the mode follows the robot if it
        // crosses a zone boundary during the command.
        Supplier<ShootingLookupTable.Mode> modeSupplier =
                () -> {
                    boolean inTrench =
                            Zones.TRENCH_ZONES.contains(robotPose).getAsBoolean()
                                    || Zones.TRENCH_DUCK_ZONES.contains(robotPose).getAsBoolean()
                                    || Zones.BUMP_ZONES.contains(robotPose).getAsBoolean();
                    return inTrench ? ShootingLookupTable.Mode.PASS : ShootingLookupTable.Mode.HUB;
                };

        // Target follows both the mode and the robot position each loop.
        Supplier<Translation3d> targetSupplier =
                () -> {
                    if (modeSupplier.get() == ShootingLookupTable.Mode.HUB)
                        return AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);
                    var pose = robotPose.get();
                    var left = FieldConstants.LeftTrench.openingTopLeft;
                    var right = FieldConstants.RightTrench.openingTopLeft;
                    double robotY = pose.getTranslation().getY();
                    return Math.abs(robotY - left.getY()) < Math.abs(robotY - right.getY())
                            ? AllianceFlipUtil.apply(left)
                            : AllianceFlipUtil.apply(right);
                };

        // Because the lookup table must be fixed at construction time, use HUB as
        // the table — realShoot is rarely used in practice; prefer aimAndShoot or
        // aimAndShootTrench for explicit mode control.
        return aimAndShoot(robotPose, fieldSpeeds, targetSupplier, 3, ShootingLookupTable.Mode.HUB)
                .withName("RealShoot");
    }

    /**
     * Aim the turret and hood, spin the flywheel to the computed speed, and feed when the hub shift
     * is active.
     *
     * <p>Feeding is shift-gated: it stops automatically when the scoring window closes and resumes
     * when the window reopens — as long as the trigger remains held.
     *
     * @param robotPose supplier of the robot pose
     * @param fieldSpeeds supplier of chassis speeds (for lead compensation)
     * @param targetSupplier supplier of the 3D target point
     * @param refinementIterations number of solver refinement iterations for the shot solution
     * @param tableMode lookup table mode (HUB or PASS)
     * @return a command that aims and feeds while scheduled
     */
    public Command aimAndShoot(
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> fieldSpeeds,
            Supplier<Translation3d> targetSupplier,
            int refinementIterations,
            ShootingLookupTable.Mode tableMode) {
        var table = makeLookupTable(tableMode);
        var solution =
                makeSolutionSupplier(robotPose, fieldSpeeds, targetSupplier, refinementIterations, table);

        // Turret tracking intentionally held at 0° until full turret control is
        // validated on hardware — swap the commented line to enable azimuth tracking.
        var turretCmd = turret.setAngle(Degrees.of(0));
        // var turretCmd = turret.setAngle(() -> solution.get().turretAzimuth());
        var hoodCmd = hood.setAngle(() -> solution.get().hoodAngle());
        var flywheelCmd = flywheel.setVelocity(makeFlywheelModelSupplier(solution));
        var feedCmd = makeFeedSequence(solution);
        var telemetryCmd = makeTelemetryCmd(robotPose, solution);

        return Commands.parallel(turretCmd.alongWith(hoodCmd), flywheelCmd, feedCmd, telemetryCmd)
                .withName("AimAndShoot");
    }

    /**
     * Aim the turret only (no feed, no flywheel). Useful for verifying turret tracking in test mode.
     *
     * @param robotPose supplier of the robot pose
     * @param fieldSpeeds supplier of chassis speeds (for lead compensation)
     * @param targetSupplier supplier of the 3D target point
     * @param refinementIterations number of solver refinement iterations
     * @param tableMode lookup table mode (HUB or PASS)
     * @return a command that tracks the computed turret azimuth while scheduled
     */
    public Command aim(
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> fieldSpeeds,
            Supplier<Translation3d> targetSupplier,
            int refinementIterations,
            ShootingLookupTable.Mode tableMode) {
        var table = makeLookupTable(tableMode);
        var solution =
                makeSolutionSupplier(robotPose, fieldSpeeds, targetSupplier, refinementIterations, table);
        return turret.setAngle(() -> solution.get().turretAzimuth()).withName("Aim");
    }

    /**
     * Aim, spin up the flywheel, and feed for a trench/pass shot.
     *
     * <p>Automatically selects the nearest trench opening (left or right) from the robot's current Y
     * position each loop and uses the PASS lookup table. Feeding is <em>not</em> shift-gated — fires
     * freely while the trigger is held regardless of hub shift state.
     *
     * @param robotPose supplier of the robot pose
     * @param fieldSpeeds supplier of chassis speeds (for lead compensation)
     * @param refinementIterations number of solver refinement iterations
     * @return a command that aims and feeds a trench shot while scheduled
     */
    public Command aimAndShootTrench(
            Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldSpeeds, int refinementIterations) {
        var table = makeLookupTable(ShootingLookupTable.Mode.PASS);

        Supplier<Translation3d> targetSupplier =
                () -> {
                    var left = FieldConstants.LeftTrench.openingTopLeft;
                    var right = FieldConstants.RightTrench.openingTopLeft;
                    double robotY = robotPose.get().getY();
                    return Math.abs(robotY - left.getY()) < Math.abs(robotY - right.getY())
                            ? AllianceFlipUtil.apply(left)
                            : AllianceFlipUtil.apply(right);
                };

        var solution =
                makeSolutionSupplier(robotPose, fieldSpeeds, targetSupplier, refinementIterations, table);

        var turretCmd = turret.setAngle(() -> solution.get().turretAzimuth());
        var hoodCmd = hood.setAngle(() -> solution.get().hoodAngle());
        var flywheelCmd = flywheel.setVelocity(makeFlywheelModelSupplier(solution));
        var feedCmd = makeFeedSequenceUngated(solution);
        var telemetryCmd = makeTelemetryCmd(robotPose, solution);

        return Commands.parallel(turretCmd.alongWith(hoodCmd), flywheelCmd, feedCmd, telemetryCmd)
                .withName("TrenchShoot");
    }

    /** Clear and unjam the full shooter pipeline (flywheel, kicker, spindexer). */
    public Command clearShooterSystem() {
        return flywheel
                .clearFlywheel()
                .alongWith(kicker.clearKicker().alongWith(spindexer.clearSpindexer()));
    }

    /**
     * Interrupt any active shooting commands without commanding zero setpoints (mechanisms coast).
     * Use {@code stopNow()} / {@code stopHold()} on individual subsystems for explicit stops.
     */
    public Command interruptShooting() {
        return Commands.runOnce(() -> {}, flywheel, kicker, spindexer, turret, hood)
                .withName("InterruptShooting");
    }

    /** Stop flywheel, kicker, and spindexer immediately. */
    public Command stopShooting() {
        return flywheel.stopNow().alongWith(kicker.stopNow()).alongWith(spindexer.stopNow());
    }

    // endregion

    // region Private helpers

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

    // endregion
}
