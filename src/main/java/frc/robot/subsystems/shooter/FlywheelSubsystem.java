package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/** AdvantageKit Flywheel Subsystem, capable of replaying the flywheel. */
public class FlywheelSubsystem extends SubsystemBase {
    /**
     * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to the SMC is an
     * Output. Everything coming from the SMC is an Input.
     */
    @AutoLog
    public static class FlywheelInputs {
            public AngularVelocity velocity = DegreesPerSecond.of(0);
        public AngularVelocity setpoint = DegreesPerSecond.of(0);
        public Voltage volts = Volts.of(0);
        public Current current = Amps.of(0);
    }

    private final FlywheelInputsAutoLogged flywheelInputs = new FlywheelInputsAutoLogged();

    private final TalonFX leftMotor = new TalonFX(51);

    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(
                            50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
                    .withSimClosedLoopController(
                            50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
                                        // Feedforward Constants
                    .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                    .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                    .withVoltageCompensation(Volts.of(12))
                    // Telemetry name and verbosity level
                    .withTelemetry("ShooterMotors", TelemetryVerbosity.HIGH)
                    // Gearing from the motor rotor to final shaft.
                    // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which
                    // corresponds to
                    // the gearbox attached to your motor.
                    // .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.COAST)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withFollowers(Pair.of(new TalonFX(52), true));

    private final SmartMotorController motor =
            new TalonFXWrapper(leftMotor, DCMotor.getKrakenX60Foc(2), motorConfig);

    private final FlyWheelConfig shooterConfig =
            new FlyWheelConfig(motor)
                    // Diameter of the flywheel.
                    .withDiameter(Inches.of(3))
                    // Mass of the flywheel.
                    .withMass(Pounds.of(3))
                    .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH);

    private final FlyWheel flywheel = new FlyWheel(shooterConfig);

    /** Update the AdvantageKit "inputs" (data coming from the SMC) */
    private void updateInputs() {
        flywheelInputs.velocity = flywheel.getSpeed();
        flywheelInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
        flywheelInputs.volts = motor.getVoltage();
        flywheelInputs.current = motor.getStatorCurrent();
    }

    public FlywheelSubsystem() {}

    /**
     * Gets the current velocity of the flywheel.
     *
     * @return FlyWheel velocity.
     */
    public AngularVelocity getVelocity() {
        return flywheelInputs.velocity;
    }

    /**
     * Set the flywheel velocity.
     *
     * @param speed Speed to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setVelocity(AngularVelocity speed) {
        Logger.recordOutput("Shooter/Setpoint", speed);
        return flywheel.setSpeed(speed);
    }

    /**
     * Set the dutycycle of the flywheel.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command set(double dutyCycle) {
        Logger.recordOutput("Shooter/DutyCycle", dutyCycle);
        return flywheel.set(dutyCycle);
    }

    public Command setVelocity(Supplier<AngularVelocity> speed) {
        return flywheel.setSpeed(
                () -> {
                    Logger.recordOutput("Shooter/Setpoint", speed.get());
                    return speed.get();
                });
    }

    public Command setDutyCycle(Supplier<Double> dutyCycle) {
        return flywheel.set(
                () -> {
                    Logger.recordOutput("Shooter/DutyCycle", dutyCycle.get());
                    return dutyCycle.get();
                });
    }

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
