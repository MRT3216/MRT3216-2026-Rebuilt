package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
// Diameter and mass are centralized in Constants.KickerConstants
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.KickerConstants.*;

import com.revrobotics.spark.SparkFlex;
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
import yams.motorcontrollers.local.SparkWrapper;

/**
 * AdvantageKit Kicker Subsystem, capable of replaying the kicker. Subsystem that manages the ball
 * kicker intake/outtake mechanism.
 */
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

    // region Lifecycle / periodic

    /** Update the AdvantageKit "inputs" (data coming from the SMC) */
    private void updateInputs() {
        kickerInputs.velocity = kicker.getSpeed();
        kickerInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
        kickerInputs.volts = motor.getVoltage();
        kickerInputs.current = motor.getStatorCurrent();
    }

    /** Construct the KickerSubsystem and configure motor/telemetry settings. */
    public KickerSubsystem() {
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
                        // Telemetry name and verbosity level
                        .withTelemetry(kKickerMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(new MechanismGearing(GearBox.fromReductionStages(kGearReduction)))
                        .withMotorInverted(false)
                        .withIdleMode(MotorMode.COAST)
                        .withStatorCurrentLimit(kStatorCurrentLimit);

        motor = new SparkWrapper(motorController, DCMotor.getNeoVortex(1), motorConfig);

        kickerConfig =
                new FlyWheelConfig(motor)
                        // Diameter of the kicker.
                        .withDiameter(kWheelDiameter)
                        // Mass of the kicker.
                        .withMass(kWheelMass)
                        // Maximum speed of the shooter.
                        // .withUpperSoftLimit(RPM.of(4000))
                        .withTelemetry(kKickerMechTelemetry, Constants.telemetryVerbosity());

        kicker = new FlyWheel(kickerConfig);
    }

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
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setVelocity(AngularVelocity speed) {
        return kicker.setSpeed(speed);
    }

    /**
     * Set the dutycycle of the kicker.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setDutyCycle(double dutyCycle) {
        // Duty cycle recorded via mechanism telemetry; removed per-invocation telemetry to reduce
        // noise.
        return kicker.set(dutyCycle);
    }

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
}
