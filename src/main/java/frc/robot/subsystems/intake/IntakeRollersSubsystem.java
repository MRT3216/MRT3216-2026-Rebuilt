package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IntakeConstants.Rollers.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;
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

/**
 * AdvantageKit-ready Intake Rollers Subsystem for MRT 3216.
 *
 * <p>This subsystem manages dual-Kraken intake rollers using the YAMS library and Phoenix 6. It
 * utilizes an IO-layer abstraction for full log replay capabilities, ensuring that hardware states
 * (Inputs) are separated from software commands (Outputs).
 */
public class IntakeRollersSubsystem extends SubsystemBase {

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

    // Explicit Phoenix refreshes are required for telemetry; call directly.

    /* Hardware Objects */
    private final TalonFX leftMotor = new TalonFX(RobotMap.Intake.Roller.kMotorId);

    /* Phoenix 6 Status Signals (for high-frequency synchronized logging) */
    private final StatusSignal<AngularVelocity> velocitySignal = leftMotor.getVelocity();
    private final StatusSignal<Double> referenceSignal = leftMotor.getClosedLoopReference();

    /* Configuration for the Smart Motor Controller (SMC) */
    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController motor;

    /* High-level mechanism configuration */
    private final FlyWheelConfig intakeRollersConfig;

    private final FlyWheel intakeRollers;

    /**
     * Updates the AdvantageKit "inputs" by refreshing hardware signals. Synchronizes TalonFX signals
     * to ensure telemetry is time-aligned.
     */
    private void updateInputs() {
        // Refresh Phoenix signals to ensure telemetry is up-to-date for AdvantageKit/YAMS
        BaseStatusSignal.refreshAll(velocitySignal, referenceSignal);

        intakeRollersInputs.velocity = intakeRollers.getSpeed();
        intakeRollersInputs.volts = motor.getVoltage();
        intakeRollersInputs.current = motor.getStatorCurrent();

        // Sets the setpoint input based on the current SMC state
        intakeRollersInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
    }

    /** Initializes the subsystem, sets signal update frequencies, and optimizes CAN utilization. */
    public IntakeRollersSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants)
                        .withClosedLoopController(kP, kI, kD)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim)
                        // Feedforward Constants
                        .withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
                        .withSimFeedforward(new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim))
                        // Telemetry
                        .withTelemetry(kIntakeRollersMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(new MechanismGearing(GearBox.fromReductionStages(kGearReduction)))
                        .withMotorInverted(false)
                        .withIdleMode(MotorMode.COAST)
                        .withStatorCurrentLimit(kStatorCurrentLimit);

        motor = new TalonFXWrapper(leftMotor, DCMotor.getKrakenX60Foc(2), motorConfig);

        intakeRollersConfig =
                new FlyWheelConfig(motor)
                        .withDiameter(kWheelDiameter)
                        .withMass(kWheelMass)
                        .withTelemetry(kIntakeRollersMechTelemetry, Constants.telemetryVerbosity());

        intakeRollers = new FlyWheel(intakeRollersConfig);

        // High-frequency updates for PID tuning (use centralized telemetry constant)
        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.CommsConstants.DEFAULT_TELEMETRY_HZ, velocitySignal, referenceSignal);

        // Optimization: Disable unused signals to conserve CAN bus bandwidth
        leftMotor.getPosition().setUpdateFrequency(0);
    }

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

    @Override
    public void simulationPeriodic() {
        intakeRollers.simIterate();
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Intake/Rollers", intakeRollersInputs);
        intakeRollers.updateTelemetry();
    }
}
