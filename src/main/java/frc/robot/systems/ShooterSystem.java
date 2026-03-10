package frc.robot.systems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kClearDurationSecs;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kFlywheelPrepAngularVelocity;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kSoftLimitMax;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kSoftLimitMin;
import static frc.robot.constants.ShooterConstants.KickerConstants.kKickerClearAngularVelocity;
import static frc.robot.constants.ShooterConstants.kRefinementConvergenceEpsilon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.SpindexerSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.util.HybridTurretUtil;
import frc.robot.util.ShooterModel;
import frc.robot.util.ShootingLookupTable;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

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

    // Atomic holder for an operator-adjustable flywheel target. Commands can bump this value
    // and the supplied startShooting(Supplier) overload below can use it to drive the flywheel.
    private final AtomicReference<AngularVelocity> flywheelTarget =
            new AtomicReference<>(kFlywheelPrepAngularVelocity);

    // endregion

    // region Public API (queries & commands)

    /**
     * High-level shoot command: spin the flywheel to the configured target, run a short clearing
     * routine, then feed balls using the kicker and spindexer.
     *
     * @return a composed command that owns the shooter subsystems while executing the shooting
     *     pipeline
     */
    public Command shootForTuning() {
        return feedAndShoot().withName("ShootForTuning");
    }

    /**
     * Start shooting with a dynamic velocity supplier. The returned command schedules the closed-loop
     * flywheel velocity command in parallel with a clear->feed sequence. The flywheel command will
     * remain active until interrupted (so it continues to adjust while feeding).
     */
    public Command startShooting(Supplier<AngularVelocity> velocitySupplier) {
        var flywheelCmd = flywheel.setVelocity(velocitySupplier);
        var feedSeq =
                Commands.sequence(clearKicker(), spindexer.feedShooter().alongWith(kicker.feedShooter()));
        return Commands.parallel(flywheelCmd, feedSeq).withName("StartShooting");
    }

    /**
     * Start shooting using the system-owned adjustable target.
     *
     * <p>This command uses an internal AtomicReference target that operator bump commands modify. It
     * schedules a re-applier that continuously writes the latest target to the flywheel so
     * adjustments take effect live during tuning. The feeding sequence (clear->feed) runs in
     * parallel.
     *
     * @return a composed Command that follows the current system-owned flywheel target and feeds
     */
    public Command startShootingWithAdjustableTarget() {
        // Start a followTarget re-applier so the adjustable atomic target is
        // continuously applied while the command is active. This guarantees live
        // updates from bump commands during tuning.
        var flywheelFollow = flywheel.followTarget(() -> flywheelTarget.get());
        var feedSeq =
                Commands.sequence(clearKicker(), spindexer.feedShooter().alongWith(kicker.feedShooter()));
        return Commands.parallel(flywheelFollow, feedSeq).withName("StartShootingAdjustable");
    }

    /**
     * Bump the system-owned flywheel target by the given delta (AngularVelocity). Clamped to soft
     * limits.
     */
    public Command bumpFlywheelTarget(AngularVelocity delta) {
        // Do not require any subsystems so bumping does not interrupt running feed/clear
        // commands. The followTarget command (if active) will pick up the new value.
        return Commands.runOnce(
                        () -> {
                            double current = flywheelTarget.get().in(RPM);
                            double d = delta.in(RPM);
                            double min = kSoftLimitMin.in(RPM);
                            double max = kSoftLimitMax.in(RPM);
                            double next = Math.max(min, Math.min(max, current + d));
                            flywheelTarget.set(RPM.of(next));
                        })
                .withName("BumpFlywheelSys");
    }

    /** Convenience factories for small integer RPM bumps (useful for button bindings). */
    /**
     * Convenience: one-shot bump of the system-owned flywheel target upward by the given RPM. This
     * command does not require shooter subsystems so it won't interrupt active shooting pipelines.
     *
     * @param rpm amount in RPM to add to the current target
     * @return a one-shot Command that bumps the target up
     */
    public Command bumpFlywheelUp(double rpm) {
        return bumpFlywheelTarget(RPM.of(rpm));
    }

    /**
     * Convenience: one-shot bump of the system-owned flywheel target downward by the given RPM. This
     * command does not require shooter subsystems so it won't interrupt active shooting pipelines.
     *
     * @param rpm amount in RPM to subtract from the current target (positive value expected)
     * @return a one-shot Command that bumps the target down
     */
    public Command bumpFlywheelDown(double rpm) {
        return bumpFlywheelTarget(RPM.of(-Math.abs(rpm)));
    }

    /** Start shooting at the canonical prep velocity (fixed-speed convenience overload). */
    public Command startShooting() {
        return startShooting(() -> kFlywheelPrepAngularVelocity);
    }

    private Command feedAndShoot() {
        // Start the flywheel closed-loop controller (YAMS run), run the short
        // clear routine, then start feeding immediately after the clear completes.
        // We intentionally do not wait for the flywheel to reach speed.
        return flywheel
                .setVelocity(kFlywheelPrepAngularVelocity)
                .andThen(clearKicker())
                .andThen(spindexer.feedShooter().alongWith(kicker.feedShooter()))
                .withName("FeedAndShoot");
    }

    /**
     * Clear the shooter: run the kicker and spindexer briefly in reverse to remove jams.
     *
     * @return a command that executes the clear routine
     */
    private Command clearKicker() {
        // Use small negative closed-loop velocities to clear any jammed balls.
        // Closed-loop ensures repeatable behavior across real and sim.
        return kicker
                .setVelocity(kKickerClearAngularVelocity)
                .withTimeout(kClearDurationSecs)
                // After the clear completes, stop the kicker using a short one-shot
                // provided by the subsystem so the sequence can progress to feeding
                // without being blocked by a long-running zero command.
                .andThen(kicker.stopNow())
                .withName("ClearKicker");
    }

    /**
     * Prepare the shooter by spinning the flywheel to the canonical target velocity while running the
     * short kicker clear routine.
     *
     * <p>This operator-facing "prep" command starts the flywheel closed-loop controller with the
     * canonical prep velocity ({@link
     * frc.robot.constants.ShooterConstants.FlywheelConstants#kFlywheelPrepAngularVelocity}).
     *
     * @return a command that begins flywheel spin-up and runs the kicker clear routine
     */
    public Command prepShooter() {
        return flywheel.setVelocity(kFlywheelPrepAngularVelocity).withName("PrepShooter");
    }
    /**
     * This composed helper is a small convenience for operator bindings: it starts the flywheel
     * spinning to the canonical target and concurrently runs the brief reverse-kicker clear routine.
     */
    /**
     * Dynamically aim turret and hood using the provided robot pose, chassis speeds, and target.
     * Returns a command that computes a ShotSolution, moves turret & hood, spins the flywheel, and
     * feeds when executing. This implementation uses HybridTurretUtil to compute motion-compensated
     * shot solutions and runs the flywheel re-applier so setpoints update continuously.
     */
    public Command aimAndShoot(
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> fieldSpeeds,
            Supplier<Translation3d> targetSupplier,
            int refinementIterations,
            ShootingLookupTable.Mode tableMode) {
        // Build a lookup table for the requested mode
        var table = new ShootingLookupTable(tableMode);

        // Supplier that computes a live ShotSolution (includes turret azimuth, hood angle, and
        // flywheel speed) using the hybrid turret util which accounts for robot motion.
        Supplier<HybridTurretUtil.ShotSolution> solutionSupplier =
                () ->
                        HybridTurretUtil.computeMovingShot(
                                robotPose.get(),
                                fieldSpeeds.get(),
                                targetSupplier.get(),
                                refinementIterations,
                                kRefinementConvergenceEpsilon,
                                table);

        // Commands to track dynamic targets for turret and hood.
        var turretCmd = turret.setAngle(() -> solutionSupplier.get().turretAzimuth());
        var hoodCmd = hood.setAngle(() -> solutionSupplier.get().hoodAngle());

        // Flywheel re-applier follows a lightweight two-point linear model derived from
        // the computed lead distance. We keep the LUT-derived hood angle & ToF but prefer
        // the simple model for flywheel velocity to make on-robot tuning faster.
        Supplier<AngularVelocity> flywheelModelSupplier =
                () -> ShooterModel.flywheelSpeedForDistance(solutionSupplier.get().leadDistance());
        var flywheelFollow = flywheel.followTarget(flywheelModelSupplier);

        // Feeding sequence runs alongside aiming and flywheel follow.
        var feedSeq =
                Commands.sequence(clearKicker(), spindexer.feedShooter().alongWith(kicker.feedShooter()));

        // Telemetry: publish model vs LUT values while aiming. Gate to Test mode so we don't
        // spam NetworkTables during competition operation.
        var telemetryCmd =
                Commands.run(
                                () -> {
                                    // Publish in Test mode or when running SIM so telemetry is available
                                    // during simulation-based tuning.
                                    if (!(DriverStation.isTest() || Constants.getMode() == Constants.Mode.SIM))
                                        return;
                                    var sol = solutionSupplier.get();
                                    var tableNt = NetworkTableInstance.getDefault().getTable("ShooterTelemetry");
                                    tableNt.getEntry("leadDistanceMeters").setDouble(sol.leadDistance().in(Meters));
                                    tableNt.getEntry("lutFlywheelRPM").setDouble(sol.flywheelSpeed().in(RPM));
                                    var model = ShooterModel.flywheelSpeedForDistance(sol.leadDistance());
                                    tableNt.getEntry("modelFlywheelRPM").setDouble(model.in(RPM));
                                    tableNt.getEntry("lutHoodDegrees").setDouble(sol.hoodAngle().in(Degrees));
                                    tableNt.getEntry("lutToFSeconds").setDouble(sol.timeOfFlight().in(Seconds));
                                    tableNt.getEntry("isValid").setBoolean(sol.isValid());
                                    tableNt
                                            .getEntry("deltaRPM")
                                            .setDouble(model.in(RPM) - sol.flywheelSpeed().in(RPM));
                                })
                        .withName("ShooterTelemetryPublisher");

        // Run turret/hood aiming in parallel with the flywheel follow, feeding pipeline, and
        // telemetry publisher.
        return Commands.parallel(turretCmd.alongWith(hoodCmd), flywheelFollow, feedSeq, telemetryCmd)
                .withName("AimAndShoot");
    }

    public Command hoodAdjustCommand(Angle delta) {
        // Keep the command factory at the system level but delegate the bump operation
        // to the HoodSubsystem to centralize clamping/telemetry. This keeps ownership
        // of the factory in the system while ensuring the subsystem enforces limits.
        return hood.bumpSetpoint(delta).withName("HoodAdjustSys");
    }

    // endregion

    // region Triggers & events

    // Trigger declarations and ephemeral event-based commands for the shooter
    // system live here.

    // endregion

    // region Private helpers

    /**
     * Stops any active shooting pipeline.
     *
     * <p>Briefly requires all shooter subsystems to interrupt running pipelines, then issues one-shot
     * imperative zero-setpoint commands so mechanisms return to idle without blocking the scheduler.
     *
     * @return a Command that cancels shooting activity and brings mechanisms to a safe idle
     */
    public Command stopShooting() {
        return Commands.runOnce(() -> {}, flywheel, kicker, spindexer, turret, hood)
                // Immediately apply zero setpoints imperatively so the stop finishes quickly.
                .andThen(flywheel.stopNow())
                .andThen(spindexer.stopNow())
                .andThen(kicker.stopNow())
                .withName("StopShooting");
    }

    // endregion
}
