package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.*;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
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
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * AdvantageKit Spindexer Subsystem for MRT 3216.
 *
 * <p>This subsystem manages a single Neo Vortex spindexer using the YAMS library. It utilizes an
 * IO-layer abstraction for full log replay capabilities, ensuring that hardware states (Inputs) are
 * separated from software commands (Outputs).
 */
public class SpindexerSubsystem extends SubsystemBase {
    // region Inputs & telemetry

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

    // endregion

    // region Hardware & controller

    /*
     * Note: This subsystem uses a REV Spark (SparkFlex) wrapped by the YAMS
     * SmartMotorController (SparkWrapper). It does NOT use CTRE Phoenix
     * devices, so there are no Phoenix `StatusSignal<?>` fields and no
     * `PhoenixUtil.refresh(...)` calls are needed. Telemetry for this
     * subsystem is provided via the YAMS wrapper/getters (e.g.:
     * `motor.getVoltage()`, `motor.getStatorCurrent()`, `spindexer.getSpeed()`),
     * which are populated in `updateInputs()` and auto-logged.
     */

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

    // endregion

    // region Initialization helpers

    /** Initializes the subsystem and configures the motor controller with constants. */
    public SpindexerSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants). Spindexer is velocity-driven
                        // — prefer PID+feedforward for velocity control instead of position profiling.
                        .withClosedLoopController(kP, kI, kD)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim)
                        // Feedforward Constants (use centralized factory to avoid parameter-order mistakes)
                        .withFeedforward(motorFeedforward())
                        .withSimFeedforward(motorFeedforwardSim())
                        // Telemetry
                        .withTelemetry(kSpindexerMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(new MechanismGearing(GearBox.fromReductionStages(kGearReduction)))
                        .withMotorInverted(true)
                        .withIdleMode(MotorMode.COAST)
                        // Enable 12V voltage compensation for REV/Spark controllers. This helps
                        // closed-loop controllers remain consistent when battery voltage sags
                        // (common during matches). Note: CTRE/Phoenix TalonFX devices do not
                        // expose the same YAMS voltage-compensation API, so we only enable this
                        // for REV-driven SmartMotorController instances.
                        .withVoltageCompensation(Volts.of(12))
                        .withStatorCurrentLimit(kStatorCurrentLimit);

        motor = new SparkWrapper(motorController, DCMotor.getNEO(1), motorConfig);

        spindexerConfig =
                new FlyWheelConfig(motor)
                        .withDiameter(kWheelDiameter)
                        .withMass(kWheelMass)
                        .withTelemetry(kSpindexerMechTelemetry, Constants.telemetryVerbosity())
                        .withUpperSoftLimit(kSoftLimitMax)
                        .withLowerSoftLimit(kSoftLimitMin);

        spindexer = new FlyWheel(spindexerConfig);
    }

    // endregion

    // region Lifecycle / periodic

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

    @Override
    public void simulationPeriodic() {
        spindexer.simIterate();
    }

    // region Lifecycle / periodic (continued)

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Spindexer", spindexerInputs);
        spindexer.updateTelemetry();
    }

    // endregion

    // region Public API (queries & commands)

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
     * Supplier-backed overload for dynamic velocities (e.g., live tuning or vision-based targets).
     *
     * @param speed supplier providing the desired AngularVelocity
     * @return a Command that tracks the supplied velocity while active
     */
    public Command setVelocity(Supplier<AngularVelocity> speed) {
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

    // endregion
}
