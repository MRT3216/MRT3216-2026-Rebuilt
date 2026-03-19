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
    private IntakeStates currentState;

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

    /**
     * Deploy if stowed, then run the intake rollers.
     *
     * @return a command to perform the intake action
     */
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
     * Deploy the intake arm by driving the pivot down briefly.
     *
     * @return a command that deploys the intake
     */
    public Command deploy() {
        return intakePivot.set(-.20).withTimeout(0.3).withName("Intake.Deploy");
    }

    /**
     * Stow the intake arm if currently deployed.
     *
     * @return a command to move the intake to the stowed angle, or a no-op if already stowed
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
