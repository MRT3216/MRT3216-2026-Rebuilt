package frc.robot.subsystems.shooter;

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
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * AdvantageKit-ready Flywheel Subsystem for MRT 3216.
 *
 * <p>This subsystem manages a dual-Kraken flywheel using the YAMS library and Phoenix 6. It
 * utilizes an IO-layer abstraction for full log replay capabilities, ensuring that hardware states
 * (Inputs) are separated from software commands (Outputs).
 *
 * <p>Subsystem controlling the flywheel shooter motor and related telemetry.
 */
public class FlywheelSubsystem extends SubsystemBase {

    /**
     * AdvantageKit-visible inputs for the Flywheel subsystem. These fields are updated each loop from
     * hardware and are intended to be logged/serialized for replay.
     */
    @AutoLog
    public static class FlywheelInputs {
        /** Actual velocity of the flywheel mechanism. */
        public AngularVelocity velocity = RPM.of(0);

        /** Current target velocity requested from the motor controller. */
        public AngularVelocity setpoint = RPM.of(0);

        /** Applied voltage across the master motor. */
        public Voltage volts = Volts.of(0);

        /** Stator current draw of the master motor (useful for identifying jams). */
        public Current current = Amps.of(0);
    }

    private final FlywheelInputsAutoLogged flywheelInputs = new FlywheelInputsAutoLogged();

    /* Hardware Objects */
    private final TalonFX leftMotor = new TalonFX(RobotMap.Shooter.Flywheel.kLeftMotorId);

    /* Phoenix 6 Status Signals (for high-frequency synchronized logging) */
    private final StatusSignal<AngularVelocity> velocitySignal = leftMotor.getVelocity();
    private final StatusSignal<Double> referenceSignal = leftMotor.getClosedLoopReference();

    /* Configuration for the Smart Motor Controller (SMC) */
    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController motor;

    /* High-level mechanism configuration */
    private final FlyWheelConfig flywheelConfig;

    private final FlyWheel flywheel;

    /**
     * Updates the AdvantageKit "inputs" by refreshing hardware signals. Synchronizes TalonFX signals
     * to ensure telemetry is time-aligned.
     */
    private void updateInputs() {
        // Refresh all Phoenix 6 signals at once to minimize CAN latency jitter
        BaseStatusSignal.refreshAll(velocitySignal, referenceSignal);

        flywheelInputs.velocity = flywheel.getSpeed();
        flywheelInputs.volts = motor.getVoltage();
        flywheelInputs.current = motor.getStatorCurrent();

        // Sets the setpoint input based on the current SMC state
        flywheelInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
    }

    /** Initializes the subsystem, sets signal update frequencies, and optimizes CAN utilization. */
    public FlywheelSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants)
                        .withClosedLoopController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
                        .withSimClosedLoopController(
                                ShooterConstants.kP_sim, ShooterConstants.kI_sim, ShooterConstants.kD_sim)
                        // Feedforward Constants
                        .withFeedforward(
                                new SimpleMotorFeedforward(
                                        ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA))
                        .withSimFeedforward(
                                new SimpleMotorFeedforward(
                                        ShooterConstants.kS_sim, ShooterConstants.kV_sim, ShooterConstants.kA_sim))
                        // Telemetry
                        .withTelemetry(ShooterConstants.kMotorTelemetry, TelemetryVerbosity.HIGH)
                        .withGearing(
                                new MechanismGearing(GearBox.fromReductionStages(ShooterConstants.kGearReduction)))
                        .withMotorInverted(true)
                        .withIdleMode(MotorMode.COAST)
                        .withStatorCurrentLimit(ShooterConstants.kStatorCurrentLimit)
                        .withFollowers(Pair.of(new TalonFX(RobotMap.Shooter.Flywheel.kRightMotorId), true));

        motor = new TalonFXWrapper(leftMotor, DCMotor.getKrakenX60Foc(2), motorConfig);

        flywheelConfig =
                new FlyWheelConfig(motor)
                        .withDiameter(ShooterConstants.kWheelDiameter)
                        .withMass(ShooterConstants.kWheelMass)
                        .withTelemetry(ShooterConstants.kMechTelemetry, TelemetryVerbosity.HIGH);

        flywheel = new FlyWheel(flywheelConfig);

        // High-frequency updates for PID tuning
        BaseStatusSignal.setUpdateFrequencyForAll(
                (int) ShooterConstants.kUpdateHz, velocitySignal, referenceSignal);

        // Optimization: Disable unused signals to conserve CAN bus bandwidth
        leftMotor.getPosition().setUpdateFrequency(0);
    }

    /**
     * Gets the current velocity of the flywheel.
     *
     * @return The current AngularVelocity measured by the encoder.
     */
    public AngularVelocity getVelocity() {
        return flywheelInputs.velocity;
    }

    /**
     * Sets the target velocity for the flywheel.
     *
     * @param speed The target AngularVelocity.
     * @return A command to set and maintain the requested speed.
     */
    public Command setVelocity(AngularVelocity speed) {
        return flywheel.setSpeed(speed);
    }

    /**
     * Sets the duty cycle (percent output) for the flywheel.
     *
     * @param dutyCycle The output percentage (-1.0 to 1.0).
     * @return A command to run the flywheel at the specified duty cycle.
     */
    public Command setDutyCycle(double dutyCycle) {
        return flywheel.set(dutyCycle);
    }

    /**
     * Run the flywheel physics simulation step when the robot is in simulation. This advances the
     * internal mechanism model by one simulation tick.
     */
    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter", flywheelInputs);
        flywheel.updateTelemetry();
    }
}
