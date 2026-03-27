package frc.robot.systems;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeConstants;
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

    // region Public API — closed-loop (requires tuned PID/FF)

    /** Returns the current logical state of the intake. */
    public IntakeStates getState() {
        return currentState;
    }

    /**
     * Deploy if stowed, then run the intake rollers. The state check is deferred so it is evaluated
     * at runtime rather than at command-creation time.
     *
     * <p><b>Requires tuned pivot PID/FF.</b> For an untuned robot use {@link #dutyCycleIntake()}.
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
     * <p><b>Requires tuned pivot PID/FF.</b>
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
     * <p><b>Requires tuned pivot PID/FF.</b> For an untuned robot use {@link #dutyCycleAgitate()}.
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

    // endregion

    // region Public API — duty-cycle (no PID/FF required)

    /**
     * Deploy the intake arm using a timed duty-cycle pulse, then run the intake rollers. No pivot
     * PID/FF gains are required — the arm drops under a fixed voltage for a fixed duration.
     *
     * <p>This is the "battle-tested" approach that works without tuned gains. Once the pivot PID/FF
     * is characterized, switch callers to {@link #intake()} for positional accuracy.
     *
     * @return a command to deploy via duty-cycle and run rollers
     */
    public Command dutyCycleIntake() {
        return Commands.either(
                        intakeRollers.intakeBalls(),
                        dutyCycleDeploy().andThen(intakeRollers.intakeBalls()),
                        () -> currentState == IntakeStates.Deployed)
                .withName("Intake.DutyCycleIntake");
    }

    /**
     * Eject (reverse) the intake using duty-cycle deploy: deploy if needed and run rollers in
     * reverse. No pivot PID/FF gains are required.
     *
     * @return a command to deploy via duty-cycle and run rollers in reverse
     */
    public Command dutyCycleEject() {
        return Commands.either(
                        intakeRollers.ejectBalls(),
                        dutyCycleDeploy().andThen(intakeRollers.ejectBalls()),
                        () -> currentState == IntakeStates.Deployed)
                .withName("Intake.DutyCycleEject");
    }

    /**
     * Agitate the intake by oscillating the pivot with timed duty-cycle pulses while running the
     * rollers. No pivot PID/FF gains are required.
     *
     * <p>Uses fixed-voltage control rather than duty-cycle so that agitation strength is consistent
     * regardless of battery state. At 12 V nominal the pull-in pulse is ~1.5 V (≈ 12.5 % duty) and
     * the push-out pulse is ~0.7 V (≈ 5.8 % duty). When the command ends (button released), the
     * logical state is reset to {@link IntakeStates#Stowed} so that the follow-up deploy command
     * (typically wired to {@code onFalse}) actually fires — agitation drifts the arm inward and it
     * needs a fresh deploy pulse afterward.
     *
     * @return a repeating agitation command
     */
    public Command dutyCycleAgitate() {
        return intakeRollers
                .intakeBalls()
                .alongWith(
                        Commands.repeatingSequence(
                                intakePivot
                                        .setVoltage(Volts.of(1.5))
                                        .withTimeout(0.30)
                                        .andThen(intakePivot.setVoltage(Volts.of(-0.7)).withTimeout(0.25))))
                .finallyDo(() -> currentState = IntakeStates.Stowed)
                .withName("Intake.DutyCycleAgitate");
    }

    /**
     * Deploy the intake arm using a timed duty-cycle pulse and update the logical state. Runs the
     * pivot at −20 % for 0.3 s, which was the proven deploy method before PID/FF tuning.
     *
     * @return a command that deploys the intake via duty-cycle
     */
    public Command dutyCycleDeploy() {
        return intakePivot
                .set(-0.20)
                .withTimeout(0.3)
                .andThen(Commands.runOnce(() -> currentState = IntakeStates.Deployed))
                .withName("Intake.DutyCycleDeploy");
    }

    // endregion

    // region Public API — common

    /** Convenience command to stop the rollers (persistent hold). */
    public Command stopRollers() {
        return intakeRollers.stopHold();
    }

    // endregion
}
