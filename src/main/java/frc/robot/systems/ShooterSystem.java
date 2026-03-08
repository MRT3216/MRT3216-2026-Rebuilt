package frc.robot.systems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kClearDurationSecs;
import static frc.robot.constants.ShooterConstants.HoodConstants.kTunableHoodAngleDeg;
import static frc.robot.constants.ShooterConstants.KickerConstants.kKickerClearAngularVelocity;
import static frc.robot.constants.ShooterConstants.kRefinementConvergenceEpsilon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.SpindexerSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FuelSim;
import frc.robot.util.geometry.Zones;
import frc.robot.util.shooter.HybridTurretUtil;
import frc.robot.util.shooter.ShooterModel;
import frc.robot.util.shooter.ShootingLookupTable;
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

    // Optional FuelSim for trajectory visualization. Null when not in simulation or not registered.
    private FuelSim fuelSim = null;

    public void setFuelSim(FuelSim sim) {
        this.fuelSim = sim;
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
     * Test-mode shoot: use tunable/constant setpoints for hood and flywheel, do not aim the turret or
     * run any auto-adjustment. Runs a short clear routine and then feeds.
     */
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
        var turretCmd = turret.setAngle(() -> solutionSupplier.get().turretAzimuth());
        var hoodCmd = hood.setAngle(() -> solutionSupplier.get().hoodAngle());

        // Flywheel follow, feed sequence, and telemetry publisher composed from helpers.
        var flywheelFollow = flywheel.setVelocity(makeFlywheelModelSupplier(solutionSupplier));
        var feedSeq = makeFeedSequence(solutionSupplier);
        var telemetryCmd = makeTelemetryCmd(solutionSupplier);

        return Commands.parallel(turretCmd.alongWith(hoodCmd), flywheelFollow, feedSeq, telemetryCmd)
                .withName("AimAndShoot");
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
        return Commands.sequence(
                        clearKicker(),
                        Commands.runOnce(
                                () -> {
                                    if (fuelSim != null) {
                                        var sol = solutionSupplier.get();
                                        fuelSim.launchFromShotSolution(sol, sol.hoodAngle());
                                    }
                                }),
                        spindexer.feedShooter().alongWith(kicker.feedShooter()))
                .withName("FeedSequence");
    }

    private Command makeTelemetryCmd(Supplier<HybridTurretUtil.ShotSolution> solutionSupplier) {
        return Commands.run(
                        () -> {
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
                            tableNt.getEntry("deltaRPM").setDouble(model.in(RPM) - sol.flywheelSpeed().in(RPM));
                        })
                .withName("ShooterTelemetryPublisher");
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
    /**
     * Interrupt any active shooting commands. This cancels pipelines but does not command zero
     * setpoints (allowing flywheel/spindexer to coast). Use {@code stopNow()} / {@code stopHold()} on
     * subsystems when an explicit stop is required.
     */
    public Command interruptShooting() {
        return Commands.runOnce(() -> {}, flywheel, kicker, spindexer, turret, hood)
                .withName("InterruptShooting");
    }

    // endregion
}
