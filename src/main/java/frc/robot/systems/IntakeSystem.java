package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;

/** Aggregated intake system (high-level grouping of intake subsystems). */
public class IntakeSystem {
    public final IntakeRollersSubsystem intakeRoller;
    public final IntakePivotSubsystem intakeArm;
    public IntakeStates currentState;

    public enum IntakeStates {
        Stowed,
        Deployed
    }

    /** Constructs a new IntakeSystem with the given subsystems. */
    public IntakeSystem(IntakeRollersSubsystem intakeRoller, IntakePivotSubsystem intakeArm) {
        this.intakeRoller = intakeRoller;
        this.intakeArm = intakeArm;
        this.currentState = IntakeStates.Stowed;
    }

    public Command intake() {
        switch (currentState) {
            case Stowed:
                return this.deploy().andThen(intakeRoller.setDutyCycle(0.5));
            case Deployed:
                return intakeRoller.setDutyCycle(0.5);
        }
        return null;
    }

    public Command deploy() {
        if (currentState == IntakeStates.Stowed) {
            currentState = IntakeStates.Deployed;
            return intakeArm.setAngle(IntakeConstants.Pivot.kDeployedAngle);
        }
        return null;
    }

    public Command stow() {
        if (currentState == IntakeStates.Deployed) {
            currentState = IntakeStates.Stowed;
            return intakeArm.setAngle(IntakeConstants.Pivot.kStowedAngle);
        }
        return null;
    }

    public IntakeStates getCurrentState() {
        return currentState;
    }
}
