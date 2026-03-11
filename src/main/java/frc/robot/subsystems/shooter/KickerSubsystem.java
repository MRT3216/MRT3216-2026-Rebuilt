package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
// Diameter and mass are centralized in Constants.KickerConstants
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.KickerConstants.kD;
import static frc.robot.constants.ShooterConstants.KickerConstants.kD_sim;
import static frc.robot.constants.ShooterConstants.KickerConstants.kGearReduction;
import static frc.robot.constants.ShooterConstants.KickerConstants.kI;
import static frc.robot.constants.ShooterConstants.KickerConstants.kI_sim;
import static frc.robot.constants.ShooterConstants.KickerConstants.kP;
import static frc.robot.constants.ShooterConstants.KickerConstants.kP_sim;
import static frc.robot.constants.ShooterConstants.KickerConstants.kSoftLimitMax;
import static frc.robot.constants.ShooterConstants.KickerConstants.kSoftLimitMin;
import static frc.robot.constants.ShooterConstants.KickerConstants.kStatorCurrentLimit;
import static frc.robot.constants.ShooterConstants.KickerConstants.kTunableKickerRPM;
import static frc.robot.constants.ShooterConstants.KickerConstants.kWheelDiameter;
import static frc.robot.constants.ShooterConstants.KickerConstants.kWheelMass;
import static frc.robot.constants.ShooterConstants.KickerConstants.motorFeedforward;
import static frc.robot.constants.ShooterConstants.KickerConstants.motorFeedforwardSim;
import static frc.robot.constants.TelemetryKeys.kKickerMechTelemetry;
import static frc.robot.constants.TelemetryKeys.kKickerMotorTelemetry;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

/** Kicker subsystem: controls the kicker wheel and telemetry. */
public class KickerSubsystem extends SubsystemBase {
    // region Inputs & telemetry

    /**
     * AdvantageKit-visible inputs for the kicker mechanism. Updated each loop from the motor
     * controller and intended for replay/logging.
     */
    @AutoLog
    public static class KickerInputs {
        /** Current measured angular velocity of the kicker wheel. */
        public AngularVelocity velocity = RPM.of(0);

        /** Currently requested setpoint velocity (if any). */
        public AngularVelocity setpoint = RPM.of(0);

        /** Applied motor voltage. */
        public Voltage volts = Volts.of(0);

        /** Stator current draw for the kicker motor. */
        public Current current = Amps.of(0);
    }

    private final KickerInputsAutoLogged kickerInputs = new KickerInputsAutoLogged();

    // endregion

    // region Hardware & controller

    private final SparkFlex motorController =
            new SparkFlex(RobotMap.Shooter.Kicker.kMotorId, SparkFlex.MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig;

    private final SmartMotorController motor;

    private final FlyWheelConfig kickerConfig;

    private final FlyWheel kicker;

    // endregion

    // region Initialization helpers

    /** Construct the KickerSubsystem and configure motor/telemetry settings. */
    public KickerSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants). Kicker is velocity-controlled
                        // — use PID+feedforward rather than positional motion profiling.
                        .withClosedLoopController(kP, kI, kD)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim)
                        // Feedforward Constants (use centralized factory to avoid parameter-order
                        // mistakes)
                        .withFeedforward(motorFeedforward())
                        .withSimFeedforward(motorFeedforwardSim())
                        // Telemetry name and verbosity levelP
                        .withTelemetry(kKickerMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(new MechanismGearing(GearBox.fromReductionStages(kGearReduction)))
                        .withMotorInverted(false)
                        // Kicker should actively stop when idle, use BRAKE so zero output
                        // results in an immediate halt rather than freewheeling.
                        .withIdleMode(MotorMode.BRAKE)
                        .withVoltageCompensation(Volts.of(12))
                        .withStatorCurrentLimit(kStatorCurrentLimit);

        motor = new SparkWrapper(motorController, DCMotor.getNeoVortex(1), motorConfig);

        kickerConfig =
                new FlyWheelConfig(motor)
                        // Diameter of the kicker.
                        .withDiameter(kWheelDiameter)
                        // Mass of the kicker.
                        .withMass(kWheelMass)
                        // Configure soft limits via YAMS so the mechanism enforces them
                        .withUpperSoftLimit(kSoftLimitMax)
                        .withLowerSoftLimit(kSoftLimitMin)
                        .withTelemetry(kKickerMechTelemetry, Constants.telemetryVerbosity());

        kicker = new FlyWheel(kickerConfig);
    }

    // endregion

    // region Lifecycle / periodic

    /** Update the AdvantageKit "inputs" (data coming from the SMC) */
    private void updateInputs() {
        kickerInputs.velocity = kicker.getSpeed();
        kickerInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
        kickerInputs.volts = motor.getVoltage();
        kickerInputs.current = motor.getStatorCurrent();
    }

    // endregion

    /** Advance the kicker simulation model by one simulation tick. */
    @Override
    public void simulationPeriodic() {
        kicker.simIterate();
    }

    @Override
    public void periodic() {
        // Pull inputs, publish to AdvantageKit, and update mechanism telemetry
        updateInputs();
        Logger.processInputs("Kicker", kickerInputs);
        kicker.updateTelemetry();
    }

    // region Public API (queries & commands)

    /**
     * Gets the current velocity of the kicker.
     *
     * @return FlyWheel velocity.
     */
    public AngularVelocity getVelocity() {
        return kickerInputs.velocity;
    }

    /**
     * Set the kicker velocity.
     *
     * @param speed Speed to set.
     * @return {@link RunCommand}
     */
    public Command setVelocity(AngularVelocity speed) {
        return kicker.setSpeed(speed);
    }

    /**
     * Convenience helper: run the kicker at the configured shooter feed velocity.
     *
     * @return a Command that sets the kicker to the shooter feed speed
     */
    public Command feedShooter() {
        return setVelocity(RPM.of(kTunableKickerRPM.get()));
    }

    /**
     * Set the dutycycle of the kicker.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link RunCommand}
     */
    public Command setDutyCycle(double dutyCycle) {
        return kicker.set(dutyCycle);
    }

    public Command stopNow() {
        /**
         * One-shot stop command: immediately disables closed-loop control and sets motor output to
         * zero, then finishes. Use for imperative immediate stops (non-blocking). This does not hold
         * the subsystem at zero after completion.
         */
        return Commands.runOnce(() -> kicker.set(0), this).withName("KickerStopNow");
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
                .withName("KickerStopHold");
    }
}
