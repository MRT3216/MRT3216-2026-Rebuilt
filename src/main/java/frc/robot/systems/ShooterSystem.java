package frc.robot.systems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kClearDurationSecs;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kFlywheelDefaultVelocity;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kTunableFlywheelRPM;
import static frc.robot.constants.ShooterConstants.HoodConstants.kTunableHoodAngleDeg;
import static frc.robot.constants.ShooterConstants.KickerConstants.kKickerClearAngularVelocity;
import static frc.robot.constants.ShooterConstants.TurretConstants.kRobotToTurretTransform;
import static frc.robot.constants.ShooterConstants.kRPMFudgePercent;
import static frc.robot.constants.ShooterConstants.kRefinementConvergenceEpsilon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants.ShootMode;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.SpindexerSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.HubShiftUtil;
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
    // region Subsystems

    public final FlywheelSubsystem flywheel;
    public final KickerSubsystem kicker;
    public final SpindexerSubsystem spindexer;
    public final TurretSubsystem turret;
    public final HoodSubsystem hood;

    // endregion

    // region Constructor

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

    // region Public API

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
     * Test-mode shoot: a stationary calibration routine for building the two-point flywheel model and
     * populating the hood-angle LUT.
     *
     * <p>Behavior:
     *
     * <ul>
     *   <li><b>Turret</b> — locked at 0° so the robot faces the hub directly.
     *   <li><b>Hood</b> — driven by {@code kTunableHoodAngleDeg} on the dashboard. Adjust until balls
     *       land in the hub, then record the distance and angle into the LUT.
     *   <li><b>Flywheel RPM</b> — automatically computed from the turret-to-hub distance via {@link
     *       ShooterModel}. If you change {@code kTunableFlywheelRPM} on the dashboard (away from its
     *       default), the tunable value is used instead — this lets you manually dial in RPM for the
     *       closest/farthest shot to create the two-point model.
     *   <li><b>Distance</b> — turret-to-hub distance is logged to {@code TestShoot/distanceMeters} in
     *       NetworkTables every loop so you can read it from the dashboard.
     *   <li><b>Feed</b> — clears the kicker, then feeds (spindexer + kicker).
     * </ul>
     *
     * @param robotPose supplier of the current robot pose (needed to compute distance)
     * @return a command that runs the test-shoot routine while scheduled
     */
    public Command testShoot(Supplier<Pose2d> robotPose) {
        var turretCmd = turret.setAngle(Degrees.of(0));
        var hoodCmd = hood.setAngle(() -> Degrees.of(kTunableHoodAngleDeg.get()));

        // Compute turret-to-hub distance each loop and decide RPM source.
        Supplier<Distance> distanceSupplier =
                () -> {
                    var hub = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);
                    var tOrigin = turretOrigin(robotPose.get());
                    double dx = hub.toTranslation2d().getX() - tOrigin.getX();
                    double dy = hub.toTranslation2d().getY() - tOrigin.getY();
                    return Meters.of(Math.hypot(dx, dy));
                };

        // If the tunable has been edited away from its default, use the manual
        // override; otherwise auto-compute from the two-point model.
        var flywheelCmd =
                flywheel.setVelocity(
                        () -> {
                            double tunableRPM = kTunableFlywheelRPM.get();
                            double defaultRPM = kFlywheelDefaultVelocity.in(RPM);
                            if (Math.abs(tunableRPM - defaultRPM) > 1.0) {
                                // Manual override — use dashboard value.
                                return RPM.of(tunableRPM);
                            }
                            // Auto — two-point model from distance.
                            return ShooterModel.flywheelSpeedForDistance(distanceSupplier.get());
                        });

        var feedCmd = clearKicker().andThen(spindexer.feedShooter().alongWith(kicker.feedShooter()));

        // Telemetry: log distance and effective RPM each loop.
        var telemetryCmd =
                Commands.run(
                        () -> {
                            var dist = distanceSupplier.get();
                            Logger.recordOutput("TestShoot/distanceMeters", dist.in(Meters));
                            Logger.recordOutput("TestShoot/distanceInches", dist.in(Meters) * 39.3701);

                            double tunableRPM = kTunableFlywheelRPM.get();
                            double defaultRPM = kFlywheelDefaultVelocity.in(RPM);
                            boolean isOverride = Math.abs(tunableRPM - defaultRPM) > 1.0;
                            Logger.recordOutput("TestShoot/rpmOverrideActive", isOverride);
                            Logger.recordOutput(
                                    "TestShoot/effectiveRPM",
                                    isOverride ? tunableRPM : ShooterModel.flywheelRPMForDistance(dist));
                            Logger.recordOutput("TestShoot/modelRPM", ShooterModel.flywheelRPMForDistance(dist));
                            Logger.recordOutput("TestShoot/hoodAngleDeg", kTunableHoodAngleDeg.get());
                        });

        return Commands.parallel(turretCmd, hoodCmd, flywheelCmd, feedCmd, telemetryCmd)
                .withName("TestShoot");
    }

    /**
     * Aim the turret and hood, spin the flywheel to the computed speed, and feed when the hub shift
     * is active.
     *
     * <p>Feeding is shift-gated: it stops automatically when the scoring window closes and resumes
     * when the window reopens — as long as the trigger remains held.
     *
     * <p>The {@code shootMode} supplier controls how the shot solution is computed each loop:
     *
     * <ul>
     *   <li>{@link ShootMode#FULL} — full shoot-on-the-fly with lead compensation.
     *   <li>{@link ShootMode#STATIC_DISTANCE} — raw hub distance (no lead), turret still tracks
     *       azimuth.
     *   <li>{@link ShootMode#FULL_STATIC} — raw hub distance, turret locked at 0°. Battle-tested comp
     *       fallback.
     * </ul>
     *
     * <p>The RPM fudge factor ({@code Shooter/RPMFudgePercent}) is always applied on top of the model
     * RPM regardless of mode.
     *
     * @param robotPose supplier of the robot pose
     * @param fieldSpeeds supplier of chassis speeds (for lead compensation in FULL mode)
     * @param targetSupplier supplier of the 3D target point
     * @param refinementIterations number of solver refinement iterations for the shot solution
     * @param tableMode lookup table mode (HUB or PASS)
     * @param shootMode supplier of the current {@link ShootMode} (may change mid-command)
     * @return a command that aims and feeds while scheduled
     */
    public Command aimAndShoot(
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> fieldSpeeds,
            Supplier<Translation3d> targetSupplier,
            int refinementIterations,
            ShootingLookupTable.Mode tableMode,
            Supplier<ShootMode> shootMode) {
        var table = new ShootingLookupTable(tableMode);

        // Solution supplier that switches between moving and static solvers each loop
        // based on the current shoot mode.
        var solution =
                makeModeAwareSolutionSupplier(
                        robotPose, fieldSpeeds, targetSupplier, refinementIterations, table, shootMode);

        // Turret: tracks computed azimuth in FULL and STATIC_DISTANCE modes, locks
        // at 0° in FULL_STATIC (the proven comp fallback).
        var turretCmd =
                turret.setAngle(
                        () -> {
                            if (shootMode.get() == ShootMode.FULL_STATIC) {
                                return Degrees.of(0);
                            }
                            return solution.get().turretAzimuth();
                        });

        // NOTE: turret tracking is currently held at 0° for all modes in
        // aimAndShoot until full azimuth control is validated on hardware. The
        // commented line below enables azimuth tracking for FULL / STATIC_DISTANCE.
        // Uncomment it and remove the turretCmd above when ready.
        // var turretCmd =
        //         turret.setAngle(
        //                 () -> {
        //                     if (shootMode.get() == ShootMode.FULL_STATIC) {
        //                         return Degrees.of(0);
        //                     }
        //                     return solution.get().turretAzimuth();
        //                 });

        var hoodCmd = hood.setAngle(() -> solution.get().hoodAngle());
        var flywheelCmd = flywheel.setVelocity(() -> applyFudge(solution));
        var feedCmd = makeFeedSequence(solution);
        var telemetryCmd = makeTelemetryCmd(robotPose, solution, shootMode);

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
        var table = new ShootingLookupTable(tableMode);
        var solution =
                makeSolutionSupplier(robotPose, fieldSpeeds, targetSupplier, refinementIterations, table);
        return turret.setAngle(() -> solution.get().turretAzimuth()).withName("Aim");
    }

    /**
     * Aim, spin up the flywheel, and feed for a pass shot.
     *
     * <p>Automatically selects the nearest pass target landing zone (left or right side of our
     * alliance) from the robot's current Y position each loop and uses the PASS lookup table. Feeding
     * is <em>not</em> shift-gated — fires freely while the trigger is held regardless of hub shift
     * state.
     *
     * @param robotPose supplier of the robot pose
     * @param fieldSpeeds supplier of chassis speeds (for lead compensation)
     * @param refinementIterations number of solver refinement iterations
     * @return a command that aims and feeds a pass shot while scheduled
     */
    public Command aimAndShootPass(
            Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldSpeeds, int refinementIterations) {
        var table = new ShootingLookupTable(ShootingLookupTable.Mode.PASS);

        // Select the nearest pass target landing zone by robot Y position each loop.
        // Flip BOTH targets to the current alliance first so the comparison is in the
        // same coordinate frame as the robot pose (critical on red alliance).
        Supplier<Translation3d> targetSupplier =
                () -> {
                    var left = AllianceFlipUtil.apply(FieldConstants.PassTarget.left);
                    var right = AllianceFlipUtil.apply(FieldConstants.PassTarget.right);
                    double robotY = robotPose.get().getY();
                    return Math.abs(robotY - left.getY()) < Math.abs(robotY - right.getY()) ? left : right;
                };

        var solution =
                makeSolutionSupplier(robotPose, fieldSpeeds, targetSupplier, refinementIterations, table);

        var turretCmd = turret.setAngle(() -> solution.get().turretAzimuth());
        var hoodCmd = hood.setAngle(() -> solution.get().hoodAngle());
        var flywheelCmd = flywheel.setVelocity(() -> applyFudge(solution));
        var feedCmd = makeFeedSequenceUngated();
        var telemetryCmd = makeTelemetryCmd(robotPose, solution, () -> ShootMode.FULL);

        return Commands.parallel(turretCmd.alongWith(hoodCmd), flywheelCmd, feedCmd, telemetryCmd)
                .withName("PassShoot");
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

    /**
     * Create a memoized solution supplier that selects between the moving-shot and static-shot
     * solvers based on the current {@link ShootMode} each loop cycle.
     */
    private Supplier<HybridTurretUtil.ShotSolution> makeModeAwareSolutionSupplier(
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> fieldSpeeds,
            Supplier<Translation3d> targetSupplier,
            int refinementIterations,
            ShootingLookupTable table,
            Supplier<ShootMode> shootMode) {
        final HybridTurretUtil.ShotSolution[] cache = new HybridTurretUtil.ShotSolution[1];
        final double[] cacheTimestamp = {-1};

        return () -> {
            double now = Timer.getFPGATimestamp();
            if (cache[0] == null || now != cacheTimestamp[0]) {
                var mode = shootMode.get();
                if (mode == ShootMode.FULL) {
                    cache[0] =
                            HybridTurretUtil.computeMovingShot(
                                    robotPose.get(),
                                    fieldSpeeds.get(),
                                    targetSupplier.get(),
                                    refinementIterations,
                                    kRefinementConvergenceEpsilon,
                                    table);
                } else {
                    // STATIC_DISTANCE and FULL_STATIC both use raw distance
                    cache[0] =
                            HybridTurretUtil.computeStaticShot(robotPose.get(), targetSupplier.get(), table);
                }
                cacheTimestamp[0] = now;
            }
            return cache[0];
        };
    }

    private Supplier<HybridTurretUtil.ShotSolution> makeSolutionSupplier(
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> fieldSpeeds,
            Supplier<Translation3d> targetSupplier,
            int refinementIterations,
            ShootingLookupTable table) {
        // Memoize the solution: compute once per FPGA timestamp, then return the
        // cached value for every subsequent call in the same loop cycle.  This avoids
        // re-running the iterative solver for every parallel command (turret, hood,
        // flywheel, feed, telemetry) that reads the solution each tick.
        final HybridTurretUtil.ShotSolution[] cache = new HybridTurretUtil.ShotSolution[1];
        final double[] cacheTimestamp = {-1};

        return () -> {
            double now = Timer.getFPGATimestamp();
            if (cache[0] == null || now != cacheTimestamp[0]) {
                cache[0] =
                        HybridTurretUtil.computeMovingShot(
                                robotPose.get(),
                                fieldSpeeds.get(),
                                targetSupplier.get(),
                                refinementIterations,
                                kRefinementConvergenceEpsilon,
                                table);
                cacheTimestamp[0] = now;
            }
            return cache[0];
        };
    }

    private Command makeFeedSequence(Supplier<HybridTurretUtil.ShotSolution> solutionSupplier) {
        // Feed only while the shifted shift is active AND the solution is within LUT
        // range. If the robot is too far / too close the feed stops, preventing wasted
        // game pieces on shots that won't score.
        return Commands.sequence(
                        clearKicker(),
                        spindexer
                                .feedShooter()
                                .alongWith(kicker.feedShooter())
                                .onlyWhile(
                                        () ->
                                                HubShiftUtil.getShiftedShiftInfo().active()
                                                        && solutionSupplier.get().isValid()))
                .withName("FeedSequence");
    }

    private Command makeFeedSequenceUngated() {
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

    /**
     * Apply the RPM fudge factor to the model speed for the given solution. Returns the final
     * flywheel target: {@code modelRPM × (1 + fudge/100)}.
     */
    private static AngularVelocity applyFudge(
            Supplier<HybridTurretUtil.ShotSolution> solutionSupplier) {
        var modelSpeed = ShooterModel.flywheelSpeedForDistance(solutionSupplier.get().leadDistance());
        double fudge = kRPMFudgePercent.get();
        return RPM.of(modelSpeed.in(RPM) * (1.0 + fudge / 100.0));
    }

    /**
     * Creates a background telemetry command that logs shot solution data each loop cycle.
     *
     * @param robotPose supplier of the robot pose
     * @param solutionSupplier supplier of the current shot solution
     * @param shootMode supplier of the active shoot mode
     * @return a command that publishes shooter telemetry while scheduled
     */
    private Command makeTelemetryCmd(
            Supplier<Pose2d> robotPose,
            Supplier<HybridTurretUtil.ShotSolution> solutionSupplier,
            Supplier<ShootMode> shootMode) {
        // Rate-limit the DriverStation warning to avoid flooding at 50 Hz.
        final double WARNING_INTERVAL_SECS = 1.0;
        final double[] lastWarningTime = {0};

        return Commands.run(
                        () -> {
                            var sol = solutionSupplier.get();
                            var mode = shootMode.get();

                            // Active mode and fudge
                            Logger.recordOutput("ShooterTelemetry/shootMode", mode.name());
                            Logger.recordOutput("ShooterTelemetry/rpmFudgePercent", kRPMFudgePercent.get());

                            // Lead distance (includes motion-predicted lead in FULL mode,
                            // raw hub distance in STATIC modes)
                            Logger.recordOutput(
                                    "ShooterTelemetry/leadDistanceMeters", sol.leadDistance().in(Meters));

                            // Distance from turret origin to alliance hub center
                            var hub = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);
                            var turretXY = turretOrigin(robotPose.get());
                            double hubDx = hub.toTranslation2d().getX() - turretXY.getX();
                            double hubDy = hub.toTranslation2d().getY() - turretXY.getY();
                            Logger.recordOutput("ShooterTelemetry/hubDistanceMeters", Math.hypot(hubDx, hubDy));

                            double modelRpm = ShooterModel.flywheelSpeedForDistance(sol.leadDistance()).in(RPM);
                            double fudgedRpm = modelRpm * (1.0 + kRPMFudgePercent.get() / 100.0);
                            Logger.recordOutput("ShooterTelemetry/modelRPM", modelRpm);
                            Logger.recordOutput("ShooterTelemetry/fudgedRPM", fudgedRpm);
                            Logger.recordOutput("ShooterTelemetry/lutHoodDegrees", sol.hoodAngle().in(Degrees));
                            Logger.recordOutput("ShooterTelemetry/lutToFSeconds", sol.timeOfFlight().in(Seconds));
                            Logger.recordOutput("ShooterTelemetry/isValid", sol.isValid());

                            if (!sol.isValid()) {
                                double now = Timer.getFPGATimestamp();
                                if (now - lastWarningTime[0] >= WARNING_INTERVAL_SECS) {
                                    DriverStation.reportWarning(
                                            "Shot solution out of LUT range (lead="
                                                    + String.format("%.2f", sol.leadDistance().in(Meters))
                                                    + " m)",
                                            false);
                                    lastWarningTime[0] = now;
                                }
                            }
                        })
                .withName("ShooterTelemetryPublisher");
    }

    // endregion
}
