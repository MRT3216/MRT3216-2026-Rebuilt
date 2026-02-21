package frc.robot.systems;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kClearDurationSecs;
import static frc.robot.constants.ShooterConstants.KickerConstants.*;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kSpindexerClearAngularVelocity;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kSpindexerTargetAngularVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.SpindexerSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.util.HybridTurretUtil;
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
    // region Subsystems

    public final FlywheelSubsystem flywheel;
    public final KickerSubsystem kicker;
    public final SpindexerSubsystem spindexer;
    public final TurretSubsystem turret;
    public final HoodSubsystem hood;

    // endregion

    // region Construction

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

    // region High-level commands

    /**
     * High-level shoot command: spin the flywheel to the configured target, run a short clearing
     * routine, then feed balls using the kicker and spindexer.
     *
     * @return a composed command that owns the shooter subsystems while executing the shooting
     *     pipeline
     */
    public Command shoot() {
        // Targets — chosen as safe defaults for testing. Adjust as needed.
        final AngularVelocity flywheelTarget =
                ShooterConstants.FlywheelConstants.kFlywheelTargetAngularVelocity;

        // Start spinning the flywheel to target velocity
        Command spin = flywheel.setVelocity(flywheelTarget);
        Command clearTimed = clear().withTimeout(Seconds.of(kClearDurationSecs));

        // Run spin and clear in parallel. After the clear timeout completes, begin feeding.
        // We run the spin (long-running) and a short sequence (clearTimed -> feedCore) in
        // parallel so the flywheel spins up while the clear runs; once the clear finishes the
        // feed will start and continue until cancelled by the operator.
        Command feedCore =
                kicker
                        .setVelocity(kKickerTargetAngularVelocity)
                        .alongWith(spindexer.setVelocity(kSpindexerTargetAngularVelocity));

        Command clearThenFeed = clearTimed.andThen(feedCore);

        return spin.alongWith(clearThenFeed);
    }

    /**
     * Clear the shooter: run the kicker and spindexer briefly in reverse to remove jams.
     *
     * @return a command that executes the clear routine
     */
    public Command clear() {
        // Use small negative closed-loop velocities to clear any jammed balls. Closed-loop
        // ensures repeatable behavior across real and sim.
        return kicker
                .setVelocity(kKickerClearAngularVelocity)
                .alongWith(spindexer.setVelocity(kSpindexerClearAngularVelocity));
    }

    /**
     * Dynamically aim turret and hood using the provided robot pose, chassis speeds, and target.
     * Returns a command that computes a ShotSolution, moves turret & hood, spins the flywheel, and
     * feeds when at speed.
     */
    public Command aimAndShoot(
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> fieldSpeeds,
            Supplier<Translation3d> targetSupplier,
            int refinementIterations,
            ShootingLookupTable.Mode tableMode) {
        AtomicReference<HybridTurretUtil.ShotSolution> shotRef = new AtomicReference<>();

        ShootingLookupTable table = new ShootingLookupTable(tableMode);

        // Continuously compute the shot solution. A separate long-running command will
        // apply the computed flywheel setpoint (so we don't construct Commands each loop).
        var computeLoop =
                Commands.run(
                        () -> {
                            try {
                                HybridTurretUtil.ShotSolution sol =
                                        HybridTurretUtil.computeMovingShot(
                                                robotPose.get(),
                                                fieldSpeeds.get(),
                                                targetSupplier.get(),
                                                refinementIterations,
                                                ShooterConstants.FlywheelConstants.kRefinementConvergenceEpsilon,
                                                table);
                                shotRef.set(sol);
                            } catch (Exception ex) {
                                // Defensive: don't let an exception kill the loop; leave last solution
                            }
                        });

        // Long-running command that continuously writes the latest computed flywheel setpoint
        // into the flywheel mechanism. This owns the flywheel subsystem and uses a supplier so
        // the setpoint is updated each loop without allocating new Commands.
        Command flywheelTrack =
                flywheel.setVelocity(
                        () -> {
                            var s = shotRef.get();
                            return s != null ? s.flywheelSpeed() : flywheel.getVelocity();
                        });

        // Supplier-backed commands that read the last computed solution (with null-safety)
        Command aimTurret =
                turret.setAngle(
                        () -> {
                            var s = shotRef.get();
                            return s != null ? s.turretAzimuth() : turret.getPosition();
                        });
        Command aimHood =
                hood.setAngle(
                        () -> {
                            var s = shotRef.get();
                            return s != null ? s.hoodAngle() : hood.getPosition();
                        });

        // Condition: shot computed and flywheel has reached the configured target within the
        // error margin. We don't require turret/hood to be at setpoint before feeding;
        // they will continue to adjust while feeding occurs.
        Command waitForSpin =
                Commands.waitUntil(() -> flywheel.atSetpoint.getAsBoolean()).withTimeout(Seconds.of(3));

        // Feed using kicker and spindexer at the fixed configured feed rates. The kicker and
        // spindexer always run at fixed velocities when feeding for this robot.
        Command feed =
                kicker
                        .setVelocity(kKickerTargetAngularVelocity)
                        .alongWith(spindexer.setVelocity(kSpindexerTargetAngularVelocity))
                        .withTimeout(Seconds.of(2));

        // Run the compute loop and the flywheel tracker in parallel until the flywheel is at
        // speed (waitForSpin finishes), while also running the aiming commands.
        Command computeAndAimRace =
                computeLoop.alongWith(flywheelTrack).raceWith(aimTurret, aimHood, waitForSpin);

        return computeAndAimRace.andThen(feed);
    }

    /**
     * Convenience: returns a composed command that runs the kicker and spindexer at the configured
     * feed velocities. This is a short helper used by the private feed monitor.
     */
    private Command feedBalls() {
        return kicker
                .setVelocity(kKickerTargetAngularVelocity)
                .alongWith(
                        spindexer.setVelocity(
                                frc.robot.constants.ShooterConstants.SpindexerConstants
                                        .kSpindexerTargetAngularVelocity));
    }

    /**
     * Private monitor: once the flywheel reaches the configured setpoint, start feeding and keep
     * feeding until the parent command is cancelled. This implements the "start feeding when at
     * speed, keep feeding until driver stops" behavior.
     */
    private Command feedWhenAtSpeed() {
        // Wrap the Trigger to a BooleanSupplier to avoid any ambiguous overloads
        return Commands.waitUntil(() -> flywheel.atSetpoint.getAsBoolean()).andThen(feedBalls());
    }

    /**
     * Start the full dynamic aim-and-shoot pipeline as a long-running command. This will: -
     * continuously compute shot solutions (using the lookup table) - track/apply flywheel setpoints -
     * aim turret and hood - begin feeding automatically once the flywheel reaches the commanded
     * setpoint and continue feeding until this returned command is cancelled.
     *
     * <p>The returned command owns the relevant subsystems so calling an interrupting command (for
     * example, {@link #stopShooting()}) will cancel it and return control to defaults.
     */
    public Command startShooting(
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> fieldSpeeds,
            Supplier<Translation3d> targetSupplier,
            int refinementIterations,
            ShootingLookupTable.Mode tableMode) {
        AtomicReference<HybridTurretUtil.ShotSolution> shotRef = new AtomicReference<>();

        ShootingLookupTable table = new ShootingLookupTable(tableMode);

        var computeLoop =
                Commands.run(
                        () -> {
                            try {
                                HybridTurretUtil.ShotSolution sol =
                                        HybridTurretUtil.computeMovingShot(
                                                robotPose.get(),
                                                fieldSpeeds.get(),
                                                targetSupplier.get(),
                                                refinementIterations,
                                                ShooterConstants.FlywheelConstants.kRefinementConvergenceEpsilon,
                                                table);
                                shotRef.set(sol);
                            } catch (Exception ex) {
                                // Defensive: preserve last solution
                            }
                        });

        Command flywheelTrack =
                flywheel.setVelocity(
                        () -> {
                            var s = shotRef.get();
                            return s != null ? s.flywheelSpeed() : flywheel.getVelocity();
                        });

        Command aimTurret =
                turret.setAngle(
                        () -> {
                            var s = shotRef.get();
                            return s != null ? s.turretAzimuth() : turret.getPosition();
                        });
        Command aimHood =
                hood.setAngle(
                        () -> {
                            var s = shotRef.get();
                            return s != null ? s.hoodAngle() : hood.getPosition();
                        });

        // Monitor that waits for flywheel to reach the commanded setpoint and then starts
        // feeding. The feed command runs until this whole startShooting command is cancelled.
        Command monitor = feedWhenAtSpeed();

        // Also run a short clear routine while the flywheel spins up. We race the clear
        // with the flywheel at-setpoint trigger so the clear stops early if the shooter
        // reaches speed before the clear timed duration completes.
        Command clearTimed =
                clear().withTimeout(Seconds.of(ShooterConstants.FlywheelConstants.kClearDurationSecs));
        Command clearDuringSpin = clearTimed.raceWith(Commands.waitUntil(flywheel.atSetpoint));

        // Run compute, flywheel tracking, aiming, the feed monitor, and the clear routine in
        // parallel. The returned command owns the subsystems; feeding will begin when the
        // flywheel reaches setpoint and the clear will automatically stop when the flywheel
        // reaches setpoint or the clear timeout expires.
        return computeLoop
                .alongWith(flywheelTrack)
                .alongWith(aimTurret)
                .alongWith(aimHood)
                .alongWith(monitor)
                .alongWith(clearDuringSpin);
    }

    /** Delegator: return a command that adjusts the hood by the provided delta. */
    public Command hoodAdjustCommand(Angle delta) {
        // Keep the command factory at the system level but delegate the bump operation
        // to the HoodSubsystem to centralize clamping/telemetry. This keeps ownership
        // of the factory in the system while ensuring the subsystem enforces limits.
        return hood.bumpBy(delta).withName("HoodAdjustSys");
    }

    // endregion

    // region Utilities / private helpers

    /**
     * Stops any active shooting pipeline by taking the required subsystems briefly (thus interrupting
     * long-running shooting commands) and allowing subsystem defaults to resume.
     */
    public Command stopShooting() {
        // A no-op runOnce that requires the shooter subsystems will interrupt running shooting
        // commands and then finish; subsystem default commands (which set duty to zero) will
        // take over immediately.
        return Commands.runOnce(() -> {}, flywheel, kicker, spindexer, turret, hood);
    }

    // endregion
}
