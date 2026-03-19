package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kD;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kD_sim;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kGearReduction;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kI;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kI_sim;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kP;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kP_sim;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kSoftLimitMax;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kSoftLimitMin;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kStatorCurrentLimit;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kTunableIndexerRPM;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kWheelDiameter;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.kWheelMass;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.motorFeedforward;
import static frc.robot.constants.ShooterConstants.SpindexerConstants.motorFeedforwardSim;
import static frc.robot.constants.TelemetryKeys.kSpindexerMechTelemetry;
import static frc.robot.constants.TelemetryKeys.kSpindexerMotorTelemetry;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import yams.motorcontrollers.local.SparkWrapper;

/** Spindexer subsystem (YAMS). Handles the spindexer motor and telemetry. */
public class SpindexerSubsystem extends SubsystemBase {
    // region Inputs & telemetry

    /**
     * IO inputs for the Spindexer. AutoLogged to provide synchronized data for AdvantageScope and log
     * replay.
     */
    @AutoLog
    public static class SpindexerInputs {
        /** Mechanism velocity (roller, after gear reduction). Logged in native YAMS units (RPS). */
        public AngularVelocity mechanismVelocity = RPM.of(0);
        /**
         * Motor rotor velocity (before gear reduction), derived as mechanismVelocity × kGearReduction.
         */
        public AngularVelocity motorVelocity = RPM.of(0);
        /** Closed-loop velocity setpoint sent to the motor controller (mechanism RPS). */
        public AngularVelocity setpoint = RPM.of(0);
        /** Applied voltage across the motor. */
        public Voltage volts = Volts.of(0);
        /** Stator current draw of the motor. */
        public Current current = Amps.of(0);
    }

    private final SpindexerInputsAutoLogged spindexerInputs = new SpindexerInputsAutoLogged();

    // endregion

    // region Hardware & controller

    /* Hardware Objects */
    private final SparkMax motorController =
            new SparkMax(RobotMap.Shooter.Spindexer.kMotorId, SparkFlex.MotorType.kBrushless);

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
                        .withClosedLoopController(kP, kI, kD)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim)
                        .withFeedforward(motorFeedforward())
                        .withSimFeedforward(motorFeedforwardSim())
                        .withTelemetry(kSpindexerMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(new MechanismGearing(GearBox.fromReductionStages(kGearReduction)))
                        .withMotorInverted(true)
                        .withIdleMode(MotorMode.COAST)
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
        spindexerInputs.mechanismVelocity = spindexer.getSpeed();
        // Motor velocity = mechanism velocity × gear reduction.
        // Use RotationsPerSecond (native YAMS unit) to avoid unit conversion ambiguity.
        spindexerInputs.motorVelocity =
                RotationsPerSecond.of(
                        spindexerInputs.mechanismVelocity.in(RotationsPerSecond) * kGearReduction);
        spindexerInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
        spindexerInputs.volts = motor.getVoltage();
        spindexerInputs.current = motor.getStatorCurrent();
    
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

    // endregion

    // region Public API (queries & commands)

    /**
     * Gets the current velocity of the spindexer.
     *
     * @return The current AngularVelocity measured by the encoder.
     */
    public AngularVelocity getVelocity() {
        return spindexerInputs.mechanismVelocity;
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

    /** Supplier-backed overload for dynamic/tunable speeds. */
    public Command setVelocity(java.util.function.Supplier<AngularVelocity> supplier) {
        return spindexer.setSpeed(supplier);
    }

    /**
     * Convenience helper: run the spindexer at the configured shooter feed velocity.
     *
     * @return a Command that sets the spindexer to the shooter feed speed
     */
    public Command feedShooter() {
        // Read the LoggedTunableNumber at runtime so dashboard edits apply while
        // the command is active.
        return setVelocity(() -> RPM.of(kTunableIndexerRPM.get()));
    }

    public Command clearSpindexer() {
        // Read the LoggedTunableNumber at runtime so dashboard edits apply while
        // the command is active.
        return setVelocity(() -> RPM.of(kTunableIndexerRPM.get() * -1));
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

    /**
     * Persistent stop command: while scheduled, disables closed-loop control and sets motor output to
     * zero. Use this as a long-running default so the mechanism remains at zero output while no other
     * commands are running. If the motor idle mode is COAST, this allows the mechanism to freewheel.
     */
    public Command stopHold() {
        return Commands.run(
                        () -> {
                            motor.stopClosedLoopController();
                            motor.setDutyCycle(0);
                        },
                        this)
                .withName("SpindexerStopHold");
    }

    /**
     * One-shot stop command: immediately disables closed-loop control and sets motor output to zero,
     * then finishes. Use for imperative immediate stops (non-blocking). This does not hold the
     * subsystem at zero after completion.
     */
    public Command stopNow() {
        /**
         * One-shot stop command: immediately disables closed-loop control and sets motor output to
         * zero, then finishes. Use for imperative immediate stops (non-blocking). This does not hold
         * the subsystem at zero after completion.
         */
        return Commands.runOnce(
                        () -> {
                            spindexer.set(0);
                            motor.setDutyCycle(0);
                        },
                        this)
                .withName("SpindexerStopNow");
    }

    // endregion
}
