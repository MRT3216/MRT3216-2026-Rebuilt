package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.RobotMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * AdvantageKit-ready Flywheel Subsystem for MRT 3216. *
 *
 * <p>This subsystem manages a dual-Kraken flywheel using the YAMS library and Phoenix 6. It
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
        /** Stator current draw of the master motor (useful for identifying jams). */
        public Current current = Amps.of(0);
    }

    private final IntakeRollersInputsAutoLogged intakeRollersInputs =
            new IntakeRollersInputsAutoLogged();

    /* Hardware Objects */
    private final TalonFX leftMotor = new TalonFX(RobotMap.Intake.kRollerMotorId);

    /* Phoenix 6 Status Signals (for high-frequency synchronized logging) */
    private final StatusSignal<AngularVelocity> velocitySignal = leftMotor.getVelocity();
    private final StatusSignal<Double> referenceSignal = leftMotor.getClosedLoopReference();

    /* Configuration for the Smart Motor Controller (SMC) */
    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController motor;

    /* High-level mechanism configuration (3 lb Flywheel) */
    private final FlyWheelConfig intakeRollersConfig;

    private final FlyWheel intakeRollers;

    /**
     * Updates the AdvantageKit "inputs" by refreshing hardware signals. Synchronizes TalonFX signals
     * to ensure telemetry is time-aligned.
     */
    private void updateInputs() {
        // Refresh all Phoenix 6 signals at once to minimize CAN latency jitter
        BaseStatusSignal.refreshAll(velocitySignal, referenceSignal);

        intakeRollersInputs.velocity = intakeRollers.getSpeed();
        intakeRollersInputs.volts = motor.getVoltage();
        intakeRollersInputs.current = motor.getStatorCurrent();

        // Sets the setpoint input based on the current SMC state
        intakeRollersInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
    }

    /** Initializes the subsystem, sets signal update frequencies, and optimizes CAN utilization. */
    public IntakeRollersSubsystem() {
        // Initialize motor/controller objects here to avoid leaking "this" during field
        // initialization (object-escape).
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
                        .withFeedforward(
                                new SimpleMotorFeedforward(
                                        ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA))
                        .withGearing(
                                new MechanismGearing(GearBox.fromReductionStages(ShooterConstants.kGearReduction)))
                        .withStatorCurrentLimit(ShooterConstants.kStatorCurrentLimit)
                        .withMotorInverted(true)
                        .withFollowers(Pair.of(new TalonFX(RobotMap.Shooter.Flywheel.kRightMotorId), true));

        // Create the SMC wrapper
        motor = new TalonFXWrapper(leftMotor, DCMotor.getKrakenX60Foc(2), motorConfig);

        // Create the mechanism config now that motor is available
        intakeRollersConfig =
                new FlyWheelConfig(motor)
                        .withDiameter(ShooterConstants.kWheelDiameter)
                        .withMass(ShooterConstants.kWheelMass);

        intakeRollers = new FlyWheel(intakeRollersConfig);

        // High-frequency updates for PID tuning
        BaseStatusSignal.setUpdateFrequencyForAll(
                (int) frc.robot.constants.Constants.ShooterConstants.kUpdateHz,
                velocitySignal,
                referenceSignal);

        // Optimization: Disable unused signals (like position) to conserve CAN bus
        // bandwidth
        leftMotor.getPosition().setUpdateFrequency(0);
    }

    /**
     * Returns the current velocity of the flywheel.
     *
     * @return The current AngularVelocity measured by the encoder.
     */
    public AngularVelocity getVelocity() {
        return intakeRollersInputs.velocity;
    }

    /**
     * Sets the target velocity for the flywheel.
     *
     * @param speed The target AngularVelocity.
     * @return A command to set and maintain the requested speed.
     */
    public Command setVelocity(AngularVelocity speed) {
        return intakeRollers.setSpeed(speed);
    }

    /**
     * Sets the duty cycle (percent output) for the flywheel.
     *
     * @param dutyCycle The output percentage (-1.0 to 1.0).
     * @return A command to run the flywheel at the specified duty cycle.
     */
    public Command setDutyCycle(double dutyCycle) {
        return intakeRollers.set(dutyCycle);
    }

    /**
     * Sets the target velocity using a dynamic supplier (e.g., from a Vision subsystem).
     *
     * @param speed A supplier providing the target AngularVelocity.
     * @return A command to track the supplier's velocity.
     */
    public Command setVelocity(Supplier<AngularVelocity> speed) {
        return intakeRollers.setSpeed(
                () -> {
                    Logger.recordOutput("IntakeRollers/Setpoint", speed.get());
                    return speed.get();
                });
    }

    /**
     * Sets the duty cycle using a dynamic supplier.
     *
     * @param dutyCycle A supplier providing the target duty cycle.
     * @return A command to track the supplier's duty cycle.
     */
    public Command setDutyCycle(Supplier<Double> dutyCycle) {
        return intakeRollers.set(
                () -> {
                    Logger.recordOutput("IntakeRollers/DutyCycle", dutyCycle.get());
                    return dutyCycle.get();
                });
    }

    @Override
    public void simulationPeriodic() {
        // Iterate physics simulation for the flywheel mechanism
        intakeRollers.simIterate();
    }

    @Override
    public void periodic() {
        // 1. Pull data from hardware
        updateInputs();

        // 2. Log data for AdvantageKit Replay
        Logger.processInputs("IntakeRollers", intakeRollersInputs);

        // 3. Log high-level outputs for AdvantageScope visual analysis
        Logger.recordOutput("IntakeRollers/FX/Velocity", velocitySignal.getValue());

        // Force conversion to RPM for standard graph scaling
        Logger.recordOutput("IntakeRollers/FX/Reference", RPM.of(referenceSignal.getValue()));

        // 4. Update YAMS telemetry for internal mechanism tracking
        intakeRollers.updateTelemetry();
    }
}
