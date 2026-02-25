package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;
import frc.robot.util.PhoenixUtil;
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
    // region Inputs & telemetry

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

    // endregion

    // region Hardware & signals

    /* Hardware Objects */
    private final TalonFX leftMotor = new TalonFX(RobotMap.Shooter.Flywheel.kLeftMotorId);

    /* Phoenix 6 Status Signals (for high-frequency synchronized logging) */
    private final StatusSignal<AngularVelocity> velocitySignal = leftMotor.getVelocity();
    private final StatusSignal<Double> referenceSignal = leftMotor.getClosedLoopReference();

    // endregion

    // region Controller configuration / mechanism

    /* Configuration for the Smart Motor Controller (SMC) */
    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController motor;

    /* High-level mechanism configuration */
    private final FlyWheelConfig flywheelConfig;

    private final FlyWheel flywheel;

    // endregion

    /**
     * Updates the AdvantageKit "inputs" by refreshing hardware signals. Synchronizes TalonFX signals
     * to ensure telemetry is time-aligned.
     */
    // region Initialization helpers

    /** Initializes the subsystem, sets signal update frequencies, and optimizes CAN utilization. */
    public FlywheelSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants)
                        .withClosedLoopController(kP, kI, kD)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim)
                        // Feedforward Constants (use centralized factory to avoid parameter-order mistakes)
                        .withFeedforward(motorFeedforward())
                        .withSimFeedforward(motorFeedforwardSim())
                        // Telemetry
                        .withTelemetry(kFlywheelMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(new MechanismGearing(GearBox.fromReductionStages(kGearReduction)))
                        .withMotorInverted(true)
                        .withIdleMode(MotorMode.COAST)
                        // NOTE: Phoenix/TalonFX devices (used here) do not currently have
                        // a YAMS-mapped voltage-compensation API we can call (unlike REV
                        // SmartMotorController wrappers). Therefore we do not call
                        // `.withVoltageCompensation(...)` for TalonFX-backed configs.
                        .withStatorCurrentLimit(kStatorCurrentLimit)
                        .withFollowers(Pair.of(new TalonFX(RobotMap.Shooter.Flywheel.kRightMotorId), true));

        motor = new TalonFXWrapper(leftMotor, DCMotor.getKrakenX60Foc(2), motorConfig);

        flywheelConfig =
                new FlyWheelConfig(motor)
                        .withDiameter(kWheelDiameter)
                        .withMass(kWheelMass)
                        .withTelemetry(kFlywheelMechTelemetry, Constants.telemetryVerbosity())
                        .withUpperSoftLimit(kSoftLimitMax)
                        .withLowerSoftLimit(kSoftLimitMin);

        flywheel = new FlyWheel(flywheelConfig);

        // High-frequency updates for PID tuning (use centralized telemetry constant)
        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.CommsConstants.HIGH_TELEMETRY_HZ, velocitySignal, referenceSignal);

        // Optimization: Disable unused signals to conserve CAN bus bandwidth
        leftMotor.getPosition().setUpdateFrequency(0);
    }

    // endregion

    // region Lifecycle / periodic

    private void updateInputs() {
        // Refresh Phoenix signals to ensure telemetry is up-to-date for AdvantageKit/YAMS
        PhoenixUtil.refresh(velocitySignal, referenceSignal);

        // Phoenix-signal logging for plotting/debug (measured vs closed-loop reference)
        Logger.recordOutput("Flywheel/FX/VelocityRPM", velocitySignal.getValue().in(RPM));
        Logger.recordOutput(
                "Flywheel/FX/ReferenceRPM",
                RPM.convertFrom(referenceSignal.getValue(), RotationsPerSecond));

        flywheelInputs.velocity = flywheel.getSpeed();
        flywheelInputs.volts = motor.getVoltage();
        flywheelInputs.current = motor.getStatorCurrent();

        // Sets the setpoint input based on the current SMC state
        flywheelInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
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
        // Record flywheel inputs under a distinct path to avoid colliding with other
        // shooter subcomponents (e.g., Turret). This organizes telemetry as
        // Shooter/Flywheel which matches other subsystem telemetry keys.
        Logger.processInputs("Shooter/Flywheel", flywheelInputs);
        flywheel.updateTelemetry();
    }

    // endregion

    // region Public API (queries & commands)

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
     * Returns a command that continuously applies the provided supplier as the flywheel setpoint.
     * This is intended for use as a long-running command that owns the flywheel subsystem and updates
     * the closed-loop target each loop without creating/scheduling commands repeatedly.
     */
    public Command setVelocity(Supplier<AngularVelocity> speed) {
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

    // endregion

    // region Triggers & events

    /**
     * Returns a Trigger that is active when the flywheel is within the configured error margin of the
     * canonical shooter target (ShooterConstants.FlywheelConstants.kFlywheelTargetAngularVelocity).
     * The Trigger evaluates the current measured velocity each time it is sampled.
     */
    /** Public Trigger active when the flywheel is within error of the canonical target. */
    public final Trigger atSpeed =
            new Trigger(
                    () -> {
                        double tgtRpm = kFlywheelTargetAngularVelocity.in(RPM);
                        return tgtRpm > 0
                                && Math.abs(getVelocity().in(RPM) - tgtRpm) <= kFlywheelAtSpeedError * tgtRpm;
                    });

    /**
     * Trigger that's active when the flywheel is within the configured error margin of the currently
     * commanded setpoint. Useful for dynamic shot targets where the setpoint changes at runtime (e.g.
     * `aimAndShoot`).
     */
    /** Public Trigger active when the flywheel is within error of the current setpoint. */
    public final Trigger atSetpoint =
            new Trigger(
                    () -> {
                        double tgtRpm = flywheelInputs.setpoint.in(RPM);
                        return tgtRpm > 0
                                && Math.abs(getVelocity().in(RPM) - tgtRpm) <= kFlywheelAtSpeedError * tgtRpm;
                    });

    // endregion
}
