package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kD;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kD_sim;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kFlywheelMechTelemetry;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kFlywheelMotorTelemetry;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kGearReduction;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kI;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kI_sim;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kP;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kP_sim;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kSoftLimitMax;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kSoftLimitMin;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kStatorCurrentLimit;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kWheelDiameter;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kWheelMass;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.motorFeedforward;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.motorFeedforwardSim;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Minimal Flywheel subsystem: callers/commands own desired setpoints. The subsystem exposes the
 * minimal YAMS-backed commands to run closed-loop velocity from an AngularVelocity or a
 * Supplier<AngularVelocity>, an open-loop duty command, and helpers to observe applied state.
 */
public class FlywheelSubsystem extends SubsystemBase {
    // region Inputs & telemetry

    @AutoLog
    public static class FlywheelInputs {
        public AngularVelocity velocity = RPM.of(0);
        public AngularVelocity setpoint = RPM.of(0);
        public Voltage volts = Volts.of(0);
        public Current current = Amps.of(0);
    }

    private final FlywheelInputsAutoLogged flywheelInputs = new FlywheelInputsAutoLogged();

    // endregion

    // region Hardware & controller

    // TODO: Add the motors in this subsystem to the CANFD bus
    private final TalonFX leftMotor = new TalonFX(RobotMap.Shooter.Flywheel.kLeftMotorId);
    private final StatusSignal<AngularVelocity> velocitySignal = leftMotor.getVelocity();
    private final StatusSignal<Double> referenceSignal = leftMotor.getClosedLoopReference();

    private final SmartMotorControllerConfig motorConfig;
    private final TalonFXWrapper motor;
    private final FlyWheelConfig flywheelConfig;
    private final FlyWheel flywheel;

    // endregion

    // region Initialization helpers

    public FlywheelSubsystem() {
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(kP, kI, kD)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim)
                        .withFeedforward(motorFeedforward())
                        .withSimFeedforward(motorFeedforwardSim())
                        .withTelemetry(kFlywheelMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(new MechanismGearing(GearBox.fromReductionStages(kGearReduction)))
                        .withMotorInverted(true)
                        .withIdleMode(MotorMode.COAST)
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

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.CommsConstants.HIGH_TELEMETRY_HZ, velocitySignal, referenceSignal);
        leftMotor.getPosition().setUpdateFrequency(0);
    }

    // endregion

    // region Lifecycle / periodic

    private void updateInputs() {
        PhoenixUtil.refresh(velocitySignal, referenceSignal);
        Logger.recordOutput("Flywheel/FX/VelocityRPM", velocitySignal.getValue().in(RPM));
        Logger.recordOutput(
                "Flywheel/FX/ReferenceRPM",
                RPM.convertFrom(referenceSignal.getValue(), RotationsPerSecond));

        flywheelInputs.velocity = flywheel.getSpeed();
        flywheelInputs.volts = motor.getVoltage();
        flywheelInputs.current = motor.getStatorCurrent();
        flywheelInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    }

    // endregion

    // region Public API (queries & commands)

    /**
     * Returns the last-measured velocity for the flywheel.
     *
     * @return measured AngularVelocity of the flywheel
     */
    public AngularVelocity getVelocity() {
        return flywheelInputs.velocity;
    }

    /**
     * Command-returning API: set the flywheel closed-loop velocity while the returned Command is
     * scheduled. Prefer this for command-based flows and button bindings.
     */
    public Command setVelocity(AngularVelocity speed) {
        return flywheel.setSpeed(speed);
    }

    /** Supplier-backed overload for dynamic/tunable speeds. */
    public Command setVelocity(Supplier<AngularVelocity> supplier) {
        return flywheel.setSpeed(supplier);
    }

    /**
     * Repeatedly reapplies the provided supplier's value imperatively while the returned command is
     * scheduled. This is useful when the underlying YAMS command does not re-evaluate a Supplier
     * after scheduling and we need a running re-applier for live tuning.
     */
    public Command followTarget(Supplier<AngularVelocity> supplier) {
        return Commands.run(() -> applySetpoint(supplier.get()), this).withName("FlywheelFollowTarget");
    }

    /**
     * Imperative API: immediately apply a mechanism velocity setpoint via YAMS.
     *
     * <p>Use only internally for short one-shot helpers (e.g., {@link #stopNow()}) or for tests. This
     * is intentionally private to avoid exposing imperative ownership in the public API.
     */
    private void applySetpoint(AngularVelocity speed) {
        flywheel.setMechanismVelocitySetpoint(speed);
    }

    /**
     * Convenience: stop the flywheel via a closed-loop zero-speed command (holds zero while
     * scheduled).
     */
    public Command stopHold() {
        return flywheel.setSpeed(RPM.of(0)).withName("FlywheelStopHold");
    }

    /**
     * Short one-shot command that imperatively applies a zero velocity setpoint and finishes. Useful
     * in sequences where an immediate non-blocking stop is required.
     */
    public Command stopNow() {
        return Commands.runOnce(() -> applySetpoint(RPM.of(0)), this).withName("FlywheelStopNow");
    }

    /** Return applied setpoint (used by default command to re-apply active setpoint). */
    public AngularVelocity getAppliedSetpoint() {
        return motor.getMechanismSetpointVelocity().orElse(flywheelInputs.setpoint);
    }
}
