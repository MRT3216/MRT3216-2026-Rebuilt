package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;

/** Aggregated intake system (high-level grouping of intake subsystems). */
public class IntakeSystem {
    // region Subsystems & state

    public final IntakeRollersSubsystem intakeRoller;
    public final IntakePivotSubsystem intakeArm;
    public IntakeStates currentState;

    // endregion

    // region Construction

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

    // region Public API

    public Command intake() {
        switch (currentState) {
            case Stowed:
                return this.deploy().andThen(intakeRoller.setDutyCycle(0.5));
            case Deployed:
                return intakeRoller.setDutyCycle(0.5);
        }
        return null;
    }

    /**
     * Deploy and run the intake rollers (or run rollers if already deployed).
     *
     * @return a command to perform the intake action
     */
    public Command deploy() {
        if (currentState == IntakeStates.Stowed) {
            currentState = IntakeStates.Deployed;
            return intakeArm.setAngle(IntakeConstants.Pivot.kDeployedAngle);
        }
        return null;
    }

    /**
     * Deploy the intake arm if currently stowed.
     *
     * @return a command to move the intake to the deployed angle, or null if already deployed
     */
    public Command stow() {
        if (currentState == IntakeStates.Deployed) {
            currentState = IntakeStates.Stowed;
            return intakeArm.setAngle(IntakeConstants.Pivot.kStowedAngle);
        }
        return null;
    }

    /**
     * Stow the intake arm if currently deployed.
     *
     * @return a command to move the intake to the stowed angle, or null if already stowed
     */
    public IntakeStates getCurrentState() {
        return currentState;
    }

    /** Returns the current intake state (Stowed or Deployed). */
}
