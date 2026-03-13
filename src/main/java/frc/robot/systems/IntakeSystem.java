package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;

/** Aggregated intake system (high-level grouping of intake subsystems). */
public class IntakeSystem {
    // region Hardware & signals

    public final IntakeRollersSubsystem intakeRollers;
    public final IntakePivotSubsystem intakePivot;
    public IntakeStates currentState;

    // endregion

    // region Initialization helpers

    public enum IntakeStates {
        Stowed,
        Deployed
    }

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

    // region Public API (queries & commands)

    public Command intake() {
        switch (currentState) {
            case Stowed:
                // Deploy then run rollers. deploy() is side-effect free and returns a Command.
                return deploy().andThen(intakeRollers.intakeBalls());
            case Deployed:
                return intakeRollers.intakeBalls();
            default:
                return Commands.none();
        }
    }

    /**
     * Eject (reverse) the intake: deploy if needed and run rollers in reverse.
     *
     * @return a command to perform the eject action
     */
    public Command eject() {
        switch (currentState) {
            case Stowed:
                return deploy().andThen(intakeRollers.ejectBalls());
            case Deployed:
                return intakeRollers.ejectBalls();
            default:
                return Commands.none();
        }
    }

    public Command agitate() {
        return intakeRollers
                .ejectBalls()
                .alongWith(
                        Commands.repeatingSequence(
                                intakePivot
                                        .set(0.15)
                                        .withTimeout(0.17)
                                        .andThen(intakePivot.set(-0.15).withTimeout(0.17))));
    }

    /**
     * Deploy and run the intake rollers (or run rollers if already deployed).
     *
     * @return a command to perform the intake action
     */
    public Command deploy() {
        if (currentState == IntakeStates.Stowed) {
            // Move the arm to the deployed angle, then update the logical state after the
            // motion
            // completes. This ensures the state reflects the physical position rather than
            // the
            // scheduled intention.
            return intakePivot
                    .set(-.30)
                    .withTimeout(0.5)
                    .andThen(Commands.runOnce(() -> currentState = IntakeStates.Deployed))
                    .withName("Intake.Deploy");
        }
        return Commands.none();
    }

    /**
     * Deploy the intake arm if currently stowed.
     *
     * @return a command to move the intake to the deployed angle, or null if already deployed
     */
    public Command stow() {
        if (currentState == IntakeStates.Deployed) {
            // Move the arm to the stowed angle, then update the logical state after motion
            // completes so other callers observe the true physical state.
            return intakePivot
                    .setAngle(IntakeConstants.Pivot.kStowedAngle)
                    .andThen(Commands.runOnce(() -> currentState = IntakeStates.Stowed))
                    .withName("Intake.Stow");
        }
        return Commands.none();
    }

    /** Convenience command to stop the rollers (persistent hold). */
    public Command stopRollers() {
        return intakeRollers.stopHold();
    }
}
