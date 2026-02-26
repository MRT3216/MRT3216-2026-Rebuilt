package frc.robot.systems;

import static frc.robot.constants.ShooterConstants.FlywheelConstants.kFlywheelPrepAngularVelocity;
import static frc.robot.constants.ShooterConstants.KickerConstants.kKickerClearAngularVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.SpindexerSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.util.ShootingLookupTable;
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
        return prepShooterWithClear();
    }

    /**
     * Clear the shooter: run the kicker and spindexer briefly in reverse to remove jams.
     *
     * @return a command that executes the clear routine
     */
    private Command clearKicker() {
        // Use small negative closed-loop velocities to clear any jammed balls.
        // Closed-loop ensures repeatable behavior across real and sim.
        return kicker.setVelocity(kKickerClearAngularVelocity);
    }

    /**
     * Prepare the shooter by spinning the flywheel to the canonical target velocity while running the
     * short kicker clear routine.
     *
     * <p>This operator-facing "prep" command starts the flywheel closed-loop controller with the
     * canonical target velocity ({@link
     * frc.robot.constants.ShooterConstants.FlywheelConstants#kFlywheelTargetAngularVelocity})
     *
     * @return a command that begins flywheel spin-up and runs the kicker clear routine
     */
    public Command prepShooter() {
        return flywheel.setVelocity(kFlywheelPrepAngularVelocity);
    }

    /**
     * This composed helper is a small convenience for operator bindings: it starts the flywheel
     * spinning to the canonical target and concurrently runs the brief reverse-kicker clear routine.
     * The flywheel continues running after the clear completes.
     */
    private Command prepShooterWithClear() {
        // Schedule the long-running flywheel spin independently so it remains running
        // after the clear routine finishes or the button is released. The clear
        // routine will end when the flywheel reaches prep speed or when the
        // configured timeout elapses (safety).
        var spin = prepShooter();
        return Commands.runOnce(
                        () -> edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(spin))
                .andThen(
                        clearKicker()
                                .withTimeout(
                                        frc.robot.constants.ShooterConstants.FlywheelConstants.kClearDurationSecs)
                                .until(flywheel.atPrepSpeed));
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
        return clearKicker();
    }

    /**
     * Delegator: return a command that adjusts the hood by the provided delta.
     *
     * <p>The system-level factory delegates the actual bump operation to the {@link HoodSubsystem} so
     * the subsystem can enforce soft/hard limits and emit telemetry consistently. The returned
     * command is named "HoodAdjustSys" to make it easy to identify in logs and dashboards.
     *
     * @param delta the angle delta to apply to the hood setpoint (can be positive or negative)
     * @return a command that bumps the hood setpoint by {@code delta}
     */
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
     * Stops any active shooting pipeline by taking the required subsystems briefly (thus interrupting
     * long-running shooting commands) and allowing subsystem defaults to resume.
     */
    public Command stopShooting() {
        // A no-op runOnce that requires the shooter subsystems will interrupt running
        // shooting
        // commands and then finish; subsystem default commands (which set duty to zero)
        // will
        // take over immediately.
        return Commands.runOnce(() -> {}, flywheel, kicker, spindexer, turret, hood);
    }

    // endregion
}
