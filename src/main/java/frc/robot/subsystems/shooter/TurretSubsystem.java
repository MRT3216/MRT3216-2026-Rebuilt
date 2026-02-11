package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.TurretConstants;
import frc.robot.constants.RobotMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {

    @AutoLog
    public static class TurretInputs {
        // Current turret angle (degrees)
        public Angle angle = Degrees.of(0);
        // Target setpoint angle (degrees)
        public Angle setpoint = Degrees.of(0);
        // Motor voltage
        public Voltage volts = Volts.of(0);
        // Motor current draw
        public Current current = Amps.of(0);
    }

    // Auto-logged inputs container used by AdvantageKit / Logger
    private final TurretInputsAutoLogged turretInputs = new TurretInputsAutoLogged();

    // CAN ID 53 - pivot motor
    private TalonFX pivotMotor = new TalonFX(RobotMap.Shooter.Turret.kPivotMotorId);

    private final SmartMotorControllerConfig motorConfig;

    private final SmartMotorController motor;

    private final PivotConfig turretConfig;

    private final Pivot turret;

    /** Update the AdvantageKit "inputs" (data coming from the SMC) */
    private void updateInputs() {
        // Pull current values from the mechanism and motor
        turretInputs.angle = turret.getAngle();
        turretInputs.setpoint = turret.getMechanismSetpoint().orElse(Degrees.of(0));
        turretInputs.volts = motor.getVoltage();
        turretInputs.current = motor.getStatorCurrent();
    }

    public TurretSubsystem() {
        // Initialize motor/controller objects in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants)
                        .withClosedLoopController(
                                TurretConstants.kP,
                                TurretConstants.kI,
                                TurretConstants.kD,
                                DegreesPerSecond.of(TurretConstants.kMaxVelocityDegPerSec),
                                DegreesPerSecondPerSecond.of(TurretConstants.kMaxAccelDegPerSec2))
                        .withSimClosedLoopController(
                                TurretConstants.kP,
                                TurretConstants.kI,
                                TurretConstants.kD,
                                DegreesPerSecond.of(TurretConstants.kMaxVelocityDegPerSec),
                                DegreesPerSecondPerSecond.of(TurretConstants.kMaxAccelDegPerSec2))
                        // Feedforward Constants
                        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                        .withTelemetry(TurretConstants.kMotorTelemetry, TelemetryVerbosity.HIGH)
                        .withGearing(TurretConstants.kGearing)
                        .withMotorInverted(TurretConstants.kMotorInverted)
                        .withIdleMode(MotorMode.BRAKE)
                        .withSoftLimit(TurretConstants.kSoftLimitMax, TurretConstants.kSoftLimitMin)
                        .withStatorCurrentLimit(TurretConstants.kStatorCurrentLimit);

        motor = new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX44Foc(1), motorConfig);

        turretConfig =
                new PivotConfig(motor)
                        .withMOI(TurretConstants.kMOI)
                        .withHardLimit(TurretConstants.kHardLimitMax, TurretConstants.kHardLimitMin)
                        .withStartingPosition(TurretConstants.kStartingPosition)
                        .withTelemetry(TurretConstants.kMechTelemetry, TelemetryVerbosity.HIGH);

        turret = new Pivot(turretConfig);
    }

    /**
     * Gets the current angle of the turret.
     *
     * @return the current turret angle as an Angle
     */
    public Angle getAngle() {
        return turret.getAngle();
    }

    public Command setAngle(Angle angle) {
        return turret.setAngle(angle);
    }

    public Command setDutyCycle(double dutyCycle) {
        return turret.set(dutyCycle);
    }

    /**
     * Creates a command that moves the turret to the provided angle setpoint.
     *
     * @param setpoint a Supplier that provides the desired Angle setpoint while the command is active
     * @return a Command that will drive the turret to the requested Angle
     */
    public Command setAngle(Supplier<Angle> setpoint) {
        return turret.setAngle(
                () -> {
                    // Log requested setpoint for debugging/telemetry
                    Logger.recordOutput("Turret/Setpoint", setpoint.get());
                    return setpoint.get();
                });
    }

    /**
     * Creates a command that runs the turret at the provided duty cycle.
     *
     * @param dutyCycle a Supplier that provides a duty cycle (-1.0 to 1.0 typically)
     * @return a Command that will drive the turret using the supplied duty cycle
     */
    public Command setDutyCycle(Supplier<Double> dutyCycle) {
        return turret.set(
                () -> {
                    // Log requested duty cycle for debugging/telemetry
                    Logger.recordOutput("Turret/DutyCycle", dutyCycle.get());
                    return dutyCycle.get();
                });
    }

    @Override
    public void simulationPeriodic() {
        // Iterate simulation model each sim tick
        turret.simIterate();
    }

    @Override
    public void periodic() {
        // Update logged inputs and telemetry each robot loop
        updateInputs();
        Logger.processInputs("Turret", turretInputs);
        turret.updateTelemetry();
    }
}
