package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;

/** Aggregated intake system (high-level grouping of intake subsystems). */
public class IntakeSystem {
    // region State

    public enum IntakeStates {
        Stowed,
        Deployed
    }

    private IntakeStates currentState;

    // endregion

    // region Subsystems

    public final IntakeRollersSubsystem intakeRollers;
    public final IntakePivotSubsystem intakePivot;

    // endregion

    // region Constructor

    /**
     * Constructs a new IntakeSystem with the given subsystems.
     *
     * @param intakeRoller the rollers subsystem used for intaking/feeding
     * @param intakeArm the pivot subsystem that deploys/stows the intake
     */
    public IntakeSystem(IntakeRollersSubsystem intakeRoller, IntakePivotSubsystem intakeArm) {
        this.intakeRollers = intakeRoller;
        this.intakePivot = intakeArm;
        this.currentState = IntakeStates.Stowed;
    }

    // endregion

    // region Public API

    /** Returns the current logical state of the intake. */
    public IntakeStates getState() {
        return currentState;
    }

    /**
     * Deploy if stowed, then run the intake rollers. The state check is deferred so it is evaluated
     * at runtime rather than at command-creation time.
     *
     * @return a command to perform the intake action
     */
    public Command intake() {
        return Commands.either(
                        intakeRollers.intakeBalls(),
                        deploy().andThen(intakeRollers.intakeBalls()),
                        () -> currentState == IntakeStates.Deployed)
                .withName("Intake.Intake");
    }

    /**
     * Eject (reverse) the intake: deploy if needed and run rollers in reverse. The state check is
     * deferred so it is evaluated at runtime.
     *
     * @return a command to perform the eject action
     */
    public Command eject() {
        return Commands.either(
                        intakeRollers.ejectBalls(),
                        deploy().andThen(intakeRollers.ejectBalls()),
                        () -> currentState == IntakeStates.Deployed)
                .withName("Intake.Eject");
    }

    /**
     * Agitate the intake by oscillating the pivot while running the rollers.
     *
     * @return a repeating agitation command
     */
    public Command agitate() {
        return intakeRollers
                .intakeBalls()
                .alongWith(
                        Commands.repeatingSequence(
                                intakePivot
                                        .set(0.15)
                                        .withTimeout(0.2)
                                        .andThen(intakePivot.set(-0.15).withTimeout(0.17))));
    }

    /**
     * Deploy the intake arm using positional control and update the logical state.
     *
     * <p>TODO: once PID/FF are tuned, verify kDeployedAngle reaches the correct physical position.
     *
     * @return a command that deploys the intake
     */
    public Command deploy() {
        return intakePivot
                .setAngle(IntakeConstants.Pivot.kDeployedAngle)
                .andThen(Commands.runOnce(() -> currentState = IntakeStates.Deployed))
                .withName("Intake.Deploy");
    }

    /**
     * Stow the intake arm using positional control and update the logical state. The state check is
     * deferred so it is evaluated at runtime.
     *
     * @return a command to move the intake to the stowed angle, or a no-op if already stowed
     */
    public Command stow() {
        return Commands.either(
                        intakePivot
                                .setAngle(IntakeConstants.Pivot.kStowedAngle)
                                .andThen(Commands.runOnce(() -> currentState = IntakeStates.Stowed))
                                .withName("Intake.Stow"),
                        Commands.none(),
                        () -> currentState == IntakeStates.Deployed)
                .withName("Intake.Stow");
    }

    /** Convenience command to stop the rollers (persistent hold). */
    public Command stopRollers() {
        return intakeRollers.stopHold();
    }

    // endregion
}
