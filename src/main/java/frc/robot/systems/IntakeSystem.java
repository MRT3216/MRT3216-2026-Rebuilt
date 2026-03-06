package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;

/** Aggregated intake system (high-level grouping of intake subsystems). */
public class IntakeSystem {
    // region Hardware & signals

    public final IntakeRollersSubsystem intakeRoller;
    public final IntakePivotSubsystem intakeArm;
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
        this.intakeRoller = intakeRoller;
        this.intakeArm = intakeArm;
        this.currentState = IntakeStates.Stowed;
    }

    // endregion

    // region Public API (queries & commands)

    public Command intake() {
        switch (currentState) {
            case Stowed:
                // Deploy then run rollers. deploy() is side-effect free and returns a Command.
                return deploy().andThen(intakeRoller.setDutyCycle(0.5));
            case Deployed:
                return intakeRoller.setDutyCycle(0.5);
            default:
                return Commands.none();
        }
    }

    /**
     * Deploy and run the intake rollers (or run rollers if already deployed).
     *
     * @return a command to perform the intake action
     */
    public Command deploy() {
        if (currentState == IntakeStates.Stowed) {
            // Return a command that updates the system state when executed, then moves the arm.
            return Commands.runOnce(() -> currentState = IntakeStates.Deployed)
                    .andThen(intakeArm.setAngle(IntakeConstants.Pivot.kDeployedAngle));
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
            // Return a command that updates the system state when executed, then moves the arm.
            return Commands.runOnce(() -> currentState = IntakeStates.Stowed)
                    .andThen(intakeArm.setAngle(IntakeConstants.Pivot.kStowedAngle));
        }
        return Commands.none();
    }
}
