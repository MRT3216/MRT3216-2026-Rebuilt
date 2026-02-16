package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.SpindexerConstants;
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
import yams.motorcontrollers.local.SparkWrapper;

/**
 * AdvantageKit Spindexer Subsystem for MRT 3216.
 *
 * <p>This subsystem manages a single Neo Vortex spindexer using the YAMS library. It utilizes an
 * IO-layer abstraction for full log replay capabilities, ensuring that hardware states (Inputs) are
 * separated from software commands (Outputs).
 */
public class SpindexerSubsystem extends SubsystemBase {

    /**
     * IO inputs for the Spindexer. AutoLogged to provide synchronized data for AdvantageScope and log
     * replay.
     */
    @AutoLog
    public static class SpindexerInputs {
        /** Actual velocity of the spindexer mechanism. */
        public AngularVelocity velocity = RPM.of(0);
        /** Current target velocity requested from the motor controller. */
        public AngularVelocity setpoint = RPM.of(0);
        /** Applied voltage across the motor. */
        public Voltage volts = Volts.of(0);
        /** Stator current draw of the motor. */
        public Current current = Amps.of(0);
    }

    private final SpindexerInputsAutoLogged spindexerInputs = new SpindexerInputsAutoLogged();

    /* Hardware Objects */
    private final SparkFlex motorController =
            new SparkFlex(RobotMap.Shooter.Spindexer.kMotorId, SparkFlex.MotorType.kBrushless);

    /* Configuration for the Smart Motor Controller (SMC) */
    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController motor;

    /* High-level mechanism configuration */
    private final FlyWheelConfig spindexerConfig;

    private final FlyWheel spindexer;

    /**
     * Updates the AdvantageKit "inputs" by reading hardware state. Provides synchronized telemetry
     * for log replay.
     */
    private void updateInputs() {
        spindexerInputs.velocity = spindexer.getSpeed();
        spindexerInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
        spindexerInputs.volts = motor.getVoltage();
        spindexerInputs.current = motor.getStatorCurrent();
    }

    /** Initializes the subsystem and configures the motor controller with constants. */
    public SpindexerSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants)
                        .withClosedLoopController(
                                SpindexerConstants.kP, SpindexerConstants.kI, SpindexerConstants.kD)
                        .withSimClosedLoopController(
                                SpindexerConstants.kP_sim, SpindexerConstants.kI_sim, SpindexerConstants.kD_sim)
                        // Feedforward Constants
                        .withFeedforward(
                                new SimpleMotorFeedforward(
                                        SpindexerConstants.kS, SpindexerConstants.kV, SpindexerConstants.kA))
                        .withSimFeedforward(
                                new SimpleMotorFeedforward(
                                        SpindexerConstants.kS_sim,
                                        SpindexerConstants.kV_sim,
                                        SpindexerConstants.kA_sim))
                        // Telemetry
                        .withTelemetry(SpindexerConstants.kMotorTelemetry, TelemetryVerbosity.HIGH)
                        .withGearing(
                                new MechanismGearing(
                                        GearBox.fromReductionStages(SpindexerConstants.kGearReduction)))
                        .withMotorInverted(true)
                        .withIdleMode(MotorMode.COAST)
                        .withStatorCurrentLimit(SpindexerConstants.kStatorCurrentLimit);

        motor = new SparkWrapper(motorController, DCMotor.getNEO(1), motorConfig);

        spindexerConfig =
                new FlyWheelConfig(motor)
                        .withDiameter(SpindexerConstants.kWheelDiameter)
                        .withMass(SpindexerConstants.kWheelMass)
                        .withTelemetry(SpindexerConstants.kMechTelemetry, TelemetryVerbosity.HIGH);

        spindexer = new FlyWheel(spindexerConfig);
    }

    /**
     * Gets the current velocity of the spindexer.
     *
     * @return The current AngularVelocity measured by the encoder.
     */
    public AngularVelocity getVelocity() {
        return spindexerInputs.velocity;
    }

    /**
     * Sets the target velocity for the spindexer.
     *
     * @param speed The target AngularVelocity.
     * @return A command to set and maintain the requested speed.
     */
    public Command setVelocity(AngularVelocity speed) {
        return spindexer.setSpeed(speed);
    }

    /**
     * Sets the duty cycle (percent output) for the spindexer.
     *
     * @param dutyCycle The output percentage (-1.0 to 1.0).
     * @return A command to run the spindexer at the specified duty cycle.
     */
    public Command setDutyCycle(double dutyCycle) {
        return spindexer.set(dutyCycle);
    }

    @Override
    public void simulationPeriodic() {
        spindexer.simIterate();
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Spindexer", spindexerInputs);
        spindexer.updateTelemetry();
    }
}
