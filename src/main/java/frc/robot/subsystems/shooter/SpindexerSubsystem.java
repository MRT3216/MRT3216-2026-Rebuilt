package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
// Diameter and mass are centralized in Constants.SpindexerConstants
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

/** AdvantageKit Spindexer Subsystem, capable of replaying the spindexer. */
public class SpindexerSubsystem extends SubsystemBase {
    /**
     * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to the SMC is an
     * Output. Everything coming from the SMC is an Input.
     */
    @AutoLog
    public static class SpindexerInputs {
        public AngularVelocity velocity = RPM.of(0);
        public AngularVelocity setpoint = RPM.of(0);
        public Voltage volts = Volts.of(0);
        public Current current = Amps.of(0);
    }

    private final SpindexerInputsAutoLogged spindexerInputs = new SpindexerInputsAutoLogged();

    private final SparkFlex motorController =
            new SparkFlex(RobotMap.Spindexer.kMotorId, SparkFlex.MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig;

    private final SmartMotorController motor;

    private final FlyWheelConfig spindexerConfig;

    private final FlyWheel spindexer;

    /** Update the AdvantageKit "inputs" (data coming from the SMC) */
    private void updateInputs() {
        spindexerInputs.velocity = spindexer.getSpeed();
        spindexerInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
        spindexerInputs.volts = motor.getVoltage();
        spindexerInputs.current = motor.getStatorCurrent();
    }

    public SpindexerSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants)
                        .withClosedLoopController(
                                SpindexerConstants.kP, SpindexerConstants.kI, SpindexerConstants.kD)
                        .withSimClosedLoopController(
                                SpindexerConstants.kP, SpindexerConstants.kI, SpindexerConstants.kD)
                        // Feedforward Constants
                        .withFeedforward(
                                new SimpleMotorFeedforward(
                                        SpindexerConstants.kS, SpindexerConstants.kV, SpindexerConstants.kA))
                        .withSimFeedforward(
                                new SimpleMotorFeedforward(
                                        SpindexerConstants.kS, SpindexerConstants.kV, SpindexerConstants.kA))
                        // .withVoltageCompensation(Volts.of(12))
                        // Telemetry name and verbosity level
                        .withTelemetry(SpindexerConstants.kMotorTelemetry, TelemetryVerbosity.HIGH)
                        .withGearing(
                                new MechanismGearing(
                                        GearBox.fromReductionStages(SpindexerConstants.kGearReduction)))
                        .withMotorInverted(false)
                        .withIdleMode(MotorMode.COAST)
                        .withStatorCurrentLimit(SpindexerConstants.kStatorCurrentLimit);

        motor = new SparkWrapper(motorController, DCMotor.getNeoVortex(1), motorConfig);

        spindexerConfig =
                new FlyWheelConfig(motor)
                        // Diameter of the spindexer.
                        .withDiameter(SpindexerConstants.kWheelDiameter)
                        // Mass of the spindexer.
                        .withMass(SpindexerConstants.kWheelMass)
                        // Maximum speed of the spindexer.
                        // .withUpperSoftLimit(RPM.of(4000))
                        .withTelemetry(SpindexerConstants.kMechTelemetry, TelemetryVerbosity.HIGH);

        spindexer = new FlyWheel(spindexerConfig);
    }

    /**
     * Gets the current velocity of the spindexer.
     *
     * @return FlyWheel velocity.
     */
    public AngularVelocity getVelocity() {
        return spindexerInputs.velocity;
    }

    /**
     * Set the spindexer velocity.
     *
     * @param speed Speed to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setVelocity(AngularVelocity speed) {
        Logger.recordOutput("Spindexer/Setpoint", speed);
        return spindexer.setSpeed(speed);
    }

    /**
     * Set the dutycycle of the spindexer.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setDutyCycle(double dutyCycle) {
        Logger.recordOutput("Spindexer/DutyCycle", dutyCycle);
        return spindexer.set(dutyCycle);
    }

    public Command setVelocity(Supplier<AngularVelocity> speed) {
        return spindexer.setSpeed(
                () -> {
                    Logger.recordOutput("Spindexer/Setpoint", speed.get());
                    return speed.get();
                });
    }

    public Command setDutyCycle(Supplier<Double> dutyCycle) {
        return spindexer.set(
                () -> {
                    Logger.recordOutput("Spindexer/DutyCycle", dutyCycle.get());
                    return dutyCycle.get();
                });
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
