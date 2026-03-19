package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IntakeConstants.Rollers.kD;
import static frc.robot.constants.IntakeConstants.Rollers.kD_sim;
import static frc.robot.constants.IntakeConstants.Rollers.kGearReduction;
import static frc.robot.constants.IntakeConstants.Rollers.kI;
import static frc.robot.constants.IntakeConstants.Rollers.kI_sim;
import static frc.robot.constants.IntakeConstants.Rollers.kMotorInverted;
import static frc.robot.constants.IntakeConstants.Rollers.kP;
import static frc.robot.constants.IntakeConstants.Rollers.kP_sim;
import static frc.robot.constants.IntakeConstants.Rollers.kSoftLimitMax;
import static frc.robot.constants.IntakeConstants.Rollers.kSoftLimitMin;
import static frc.robot.constants.IntakeConstants.Rollers.kStatorCurrentLimit;
import static frc.robot.constants.IntakeConstants.Rollers.kWheelDiameter;
import static frc.robot.constants.IntakeConstants.Rollers.kWheelMass;
import static frc.robot.constants.IntakeConstants.Rollers.motorFeedforward;
import static frc.robot.constants.IntakeConstants.Rollers.motorFeedforwardSim;
import static frc.robot.constants.TelemetryKeys.kIntakeRollersMechTelemetry;
import static frc.robot.constants.TelemetryKeys.kIntakeRollersMotorTelemetry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotMap;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

/** Intake rollers subsystem: controls dual rollers and telemetry. */
public class IntakeRollersSubsystem extends SubsystemBase {
    // region Inputs & telemetry

    /**
     * IO inputs for the Intake Rollers. AutoLogged to provide synchronized data for AdvantageScope
     * and log replay.
     */
    @AutoLog
    public static class IntakeRollersInputs {
        /** Actual velocity of the intake rollers mechanism. */
        public AngularVelocity velocity = RPM.of(0);
        /** Current target velocity requested from the motor controller. */
        public AngularVelocity setpoint = RPM.of(0);
        /** Applied voltage across the master motor. */
        public Voltage volts = Volts.of(0);
        /** Stator current draw of the master motor. */
        public Current current = Amps.of(0);
    }

    private final IntakeRollersInputsAutoLogged intakeRollersInputs =
            new IntakeRollersInputsAutoLogged();

    // endregion

    // Explicit Phoenix refreshes are required for telemetry; call directly.

    // region Hardware & controller

    /* Hardware Objects */
    private final TalonFX leftMotor = new TalonFX(RobotMap.Intake.Roller.kMotorId);

    /* Phoenix 6 Status Signals (for high-frequency synchronized logging) */
    private final StatusSignal<AngularVelocity> velocitySignal = leftMotor.getVelocity();
    private final StatusSignal<Double> referenceSignal = leftMotor.getClosedLoopReference();

    // endregion

    // region Initialization helpers

    /* Configuration for the Smart Motor Controller (SMC) */
    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController motor;

    /* High-level mechanism configuration */
    private final FlyWheelConfig intakeRollersConfig;

    private final FlyWheel intakeRollers;

    // endregion

    // region Initialization helpers

    /** Initializes the subsystem, sets signal update frequencies, and optimizes CAN utilization. */
    public IntakeRollersSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withMotorInverted(kMotorInverted)
                        .withClosedLoopController(kP, kI, kD)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim)
                        .withFeedforward(motorFeedforward())
                        .withSimFeedforward(motorFeedforwardSim())
                        .withTelemetry(kIntakeRollersMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(new MechanismGearing(GearBox.fromReductionStages(kGearReduction)))
                        // Intake rollers should stop when idle; use BRAKE to hold zero output.
                        .withIdleMode(MotorMode.BRAKE)
                        .withStatorCurrentLimit(kStatorCurrentLimit);

        motor = new TalonFXWrapper(leftMotor, DCMotor.getKrakenX60Foc(1), motorConfig);

        intakeRollersConfig =
                new FlyWheelConfig(motor)
                        .withDiameter(kWheelDiameter)
                        .withMass(kWheelMass)
                        .withUpperSoftLimit(kSoftLimitMax)
                        .withLowerSoftLimit(kSoftLimitMin)
                        .withTelemetry(kIntakeRollersMechTelemetry, Constants.telemetryVerbosity());

        intakeRollers = new FlyWheel(intakeRollersConfig);

        // High-frequency updates for PID tuning (use centralized telemetry constant)
        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.CommsConstants.DEFAULT_TELEMETRY_HZ, velocitySignal, referenceSignal);

        // Optimization: Disable unused signals to conserve CAN bus bandwidth
        leftMotor.getPosition().setUpdateFrequency(0);
    }

    // endregion

    // region Lifecycle / periodic

    /**
     * Updates the AdvantageKit "inputs" by refreshing hardware signals. Synchronizes TalonFX signals
     * to ensure telemetry is time-aligned.
     */
    private void updateInputs() {
        // Refresh Phoenix signals to ensure telemetry is up-to-date for
        // AdvantageKit/YAMS
        PhoenixUtil.refresh(velocitySignal, referenceSignal);

        // Phoenix-signal logging for debug (measured vs closed-loop reference)
        Logger.recordOutput("IntakeRollers/FX/VelocityRPM", velocitySignal.getValue().in(RPM));
        Logger.recordOutput(
                "IntakeRollers/FX/ReferenceRPM",
                RPM.convertFrom(referenceSignal.getValue(), RotationsPerSecond));

        intakeRollersInputs.velocity = intakeRollers.getSpeed();
        intakeRollersInputs.volts = motor.getVoltage();
        intakeRollersInputs.current = motor.getStatorCurrent();

        // Sets the setpoint input based on the current SMC state
        intakeRollersInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
        Logger.recordOutput(
                "Mechanisms/IntakeRollersIsMoving", Math.abs(intakeRollersInputs.velocity.in(RPM)) > 10.0);
    }

    @Override
    public void simulationPeriodic() {
        intakeRollers.simIterate();
    }

    // region Lifecycle / periodic

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Intake/Rollers", intakeRollersInputs);
        intakeRollers.updateTelemetry();
    }

    // endregion

    // region Public API (queries & commands)

    /**
     * Gets the current velocity of the intake rollers.
     *
     * @return The current AngularVelocity measured by the encoder.
     */
    public AngularVelocity getVelocity() {
        return intakeRollersInputs.velocity;
    }

    /**
     * Sets the target velocity for the intake rollers.
     *
     * @param speed The target AngularVelocity.
     * @return A command to set and maintain the requested speed.
     */
    public Command setVelocity(AngularVelocity speed) {
        return intakeRollers.setSpeed(speed);
    }

    /**
     * Sets the duty cycle (percent output) for the intake rollers.
     *
     * @param dutyCycle The output percentage (-1.0 to 1.0).
     * @return A command to run the intake rollers at the specified duty cycle.
     */
    public Command setDutyCycle(double dutyCycle) {
        return intakeRollers.set(dutyCycle);
    }

    public Command intakeBalls() {
        return intakeRollers.setSpeed(IntakeConstants.Rollers.kTargetAngularVelocity);
    }

    public Command ejectBalls() {
        return intakeRollers.set(-1.0);
    }

    /**
     * One-shot stop command: immediately disables closed-loop control and sets motor output to
     * zero,then finishes. Use for imperative immediate stops (non-blocking). This does not hold the
     * subsystem at zero after completion.
     */
    public Command stopNow() {
        return Commands.runOnce(
                        () -> {
                            motor.stopClosedLoopController();
                            motor.setDutyCycle(0);
                        },
                        this)
                .withName("IntakeRollersStopNow");
    }

    /**
     * Persistent stop command: while scheduled, disables closed-loop control and sets motor output to
     * zero. Use this as a long-running default so the mechanism remains at zero output while no other
     * commands are running.
     */
    public Command stopHold() {
        return Commands.run(
                        () -> {
                            motor.stopClosedLoopController();
                            motor.setDutyCycle(0);
                        },
                        this)
                .withName("IntakeRollersStopHold");
    }
}
