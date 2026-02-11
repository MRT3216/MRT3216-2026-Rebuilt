package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
// Diameter and mass are centralized in Constants.KickerConstants
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
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/** AdvantageKit Kicker Subsystem, capable of replaying the kicker. */
public class KickerSubsystem extends SubsystemBase {
    /**
     * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to the SMC is an
     * Output. Everything coming from the SMC is an Input.
     */
    @AutoLog
    public static class KickerInputs {
        public AngularVelocity velocity = RPM.of(0);
        public AngularVelocity setpoint = RPM.of(0);
        public Voltage volts = Volts.of(0);
        public Current current = Amps.of(0);
    }

    private final KickerInputsAutoLogged kickerInputs = new KickerInputsAutoLogged();

    private final SparkFlex motorController =
            new SparkFlex(RobotMap.Shooter.Kicker.kMotorId, SparkFlex.MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig;

    private final SmartMotorController motor;

    private final FlyWheelConfig kickerConfig;

    private final FlyWheel kicker;

    /** Update the AdvantageKit "inputs" (data coming from the SMC) */
    private void updateInputs() {
        kickerInputs.velocity = kicker.getSpeed();
        kickerInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
        kickerInputs.volts = motor.getVoltage();
        kickerInputs.current = motor.getStatorCurrent();
    }

    public KickerSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants)
                        .withClosedLoopController(
                                frc.robot.constants.Constants.KickerConstants.kP,
                                frc.robot.constants.Constants.KickerConstants.kI,
                                frc.robot.constants.Constants.KickerConstants.kD)
                        .withSimClosedLoopController(
                                frc.robot.constants.Constants.KickerConstants.kP,
                                frc.robot.constants.Constants.KickerConstants.kI,
                                frc.robot.constants.Constants.KickerConstants.kD)
                        // Feedforward Constants
                        .withFeedforward(
                                new SimpleMotorFeedforward(
                                        frc.robot.constants.Constants.KickerConstants.kS,
                                        frc.robot.constants.Constants.KickerConstants.kV,
                                        frc.robot.constants.Constants.KickerConstants.kA))
                        .withSimFeedforward(
                                new SimpleMotorFeedforward(
                                        frc.robot.constants.Constants.KickerConstants.kS,
                                        frc.robot.constants.Constants.KickerConstants.kV,
                                        frc.robot.constants.Constants.KickerConstants.kA))
                        // .withVoltageCompensation(Volts.of(12))
                        // Telemetry name and verbosity level
                        .withTelemetry(
                                frc.robot.constants.Constants.KickerConstants.kMotorTelemetry,
                                TelemetryVerbosity.HIGH)
                        .withGearing(
                                new MechanismGearing(
                                        GearBox.fromReductionStages(
                                                frc.robot.constants.Constants.KickerConstants.kGearReduction)))
                        .withMotorInverted(false)
                        .withIdleMode(MotorMode.COAST)
                        .withStatorCurrentLimit(
                                frc.robot.constants.Constants.KickerConstants.kStatorCurrentLimit);

        motor = new SparkWrapper(motorController, DCMotor.getNeoVortex(1), motorConfig);

        kickerConfig =
                new FlyWheelConfig(motor)
                        // Diameter of the kicker.
                        .withDiameter(frc.robot.constants.Constants.KickerConstants.kWheelDiameter)
                        // Mass of the kicker.
                        .withMass(frc.robot.constants.Constants.KickerConstants.kWheelMass)
                        // Maximum speed of the shooter.
                        // .withUpperSoftLimit(RPM.of(4000))
                        .withTelemetry(
                                frc.robot.constants.Constants.KickerConstants.kMechTelemetry,
                                TelemetryVerbosity.HIGH);

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
        Logger.recordOutput("Kicker/Setpoint", speed);
        return kicker.setSpeed(speed);
    }

    /**
     * Set the dutycycle of the kicker.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setDutyCycle(double dutyCycle) {
        Logger.recordOutput("Kicker/DutyCycle", dutyCycle);
        return kicker.set(dutyCycle);
    }

    public Command setVelocity(Supplier<AngularVelocity> speed) {
        return kicker.setSpeed(
                () -> {
                    Logger.recordOutput("Kicker/Setpoint", speed.get());
                    return speed.get();
                });
    }

    public Command setDutyCycle(Supplier<Double> dutyCycle) {
        return kicker.set(
                () -> {
                    Logger.recordOutput("Kicker/DutyCycle", dutyCycle.get());
                    return dutyCycle.get();
                });
    }

    @Override
    public void simulationPeriodic() {
        kicker.simIterate();
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Kicker", kickerInputs);
        kicker.updateTelemetry();
    }
}
