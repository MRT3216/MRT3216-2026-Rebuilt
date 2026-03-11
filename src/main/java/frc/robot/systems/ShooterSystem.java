package frc.robot.systems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kClearDurationSecs;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kTunableFlywheelRPM;
import static frc.robot.constants.ShooterConstants.KickerConstants.kKickerClearAngularVelocity;
import static frc.robot.constants.ShooterConstants.kRefinementConvergenceEpsilon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.SpindexerSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.HybridTurretUtil;
import frc.robot.util.ShooterModel;
import frc.robot.util.ShootingLookupTable;
import frc.robot.util.Zones;
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

    /** Start shooting at the default velocity (fixed-speed convenience overload). */
    public Command startShooting() {
        return startShooting(() -> RPM.of(kTunableFlywheelRPM.get()));
    }

    private Command feedAndShoot() {
        // Start the flywheel closed-loop controller (YAMS run), run the short
        // clear routine, then start feeding immediately after the clear completes.
        // We intentionally do not wait for the flywheel to reach speed.
        return flywheel
                .setVelocity(RPM.of(kTunableFlywheelRPM.get()))
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
     * Test-mode shoot: use tunable/constant setpoints for hood and flywheel, do not aim the turret or
     * run any auto-adjustment. Runs a short clear routine and then feeds.
     */
    public Command testShoot() {
        var hoodRun = hood.runTo(Degrees.of(ShooterConstants.HoodConstants.kTunableHoodAngleDeg.get()));
        var flywheelCmd = flywheel.setVelocity(RPM.of(kTunableFlywheelRPM.get()));

        // First move the hood to the tuned angle (this command completes when the
        // hood reaches its target), then run flywheel + clear routine in parallel
        // before feeding.
        return hoodRun
                .andThen(Commands.parallel(flywheelCmd, clearKicker()))
                .andThen(spindexer.feedShooter().alongWith(kicker.feedShooter()))
                .withName("TestShoot");
    }

    /**
     * Real shoot: automatically choose HUB vs PASS mode based on the robot pose (zones) and aim
     * accordingly. Builds a target supplier from field constants and delegates to {@link
     * #aimAndShoot(Supplier, Supplier, Supplier, int, ShootingLookupTable.Mode)}.
     */
    public Command realShoot(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldSpeeds) {
        // Determine whether we are in a trench/pass-like zone. Treat trench, trench-duck, and
        // bump collections as PASS mode; otherwise use HUB.
        boolean inTrench =
                Zones.TRENCH_ZONES.contains(robotPose).getAsBoolean()
                        || Zones.TRENCH_DUCK_ZONES.contains(robotPose).getAsBoolean()
                        || Zones.BUMP_ZONES.contains(robotPose).getAsBoolean();

        var tableMode = inTrench ? ShootingLookupTable.Mode.PASS : ShootingLookupTable.Mode.HUB;

        // Target supplier: hub center for HUB mode; nearest trench opening for PASS mode.
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
                                    if (!(Constants.tuningMode || Constants.getMode() == Constants.Mode.SIM)) return;
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
        // Bump-style adjustments were removed; instead compute a new absolute target and
        // command the hood to move to that angle and hold.
        return hood.setAngle(() -> hood.getTarget().plus(delta)).withName("HoodAdjustSys");
    }

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
        // Interrupt any active shooter pipelines and let mechanisms return to their
        // default (coast) behavior. We intentionally avoid issuing one-shot zero
        // setpoints for flywheel/spindexer so they are allowed to coast after being
        // cancelled. Kicker is left alone here (subsystems that need an explicit
        // zero can still use stopNow when required).
        return Commands.runOnce(() -> {}, flywheel, kicker, spindexer, turret, hood)
                .withName("StopShooting");
    }

    // endregion
}
