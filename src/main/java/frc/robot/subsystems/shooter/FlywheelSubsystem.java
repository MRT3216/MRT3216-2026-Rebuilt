package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kD;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kD_sim;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kGearReduction;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kI;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kI_sim;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kP;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kP_sim;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kSoftLimitMax;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kSoftLimitMin;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kStatorCurrentLimit;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kTunableFlywheelRPM;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kVelocityTolerance;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kWheelDiameter;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.kWheelMass;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.motorFeedforward;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants.motorFeedforwardSim;

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
import frc.robot.Robot;
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

/** Flywheel subsystem: velocity control, open-loop duty, and telemetry helpers. */
public class FlywheelSubsystem extends SubsystemBase {
    private static final String kFlywheelMotorTelemetry = "FlywheelMotor";
    private static final String kFlywheelMechTelemetry = "FlywheelMech";

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

    // region Hardware

    private final TalonFX leftMotor = new TalonFX(RobotMap.Shooter.Flywheel.kLeftMotorId);
    private final StatusSignal<AngularVelocity> velocitySignal = leftMotor.getVelocity();
    private final StatusSignal<Double> referenceSignal = leftMotor.getClosedLoopReference();

    // endregion

    // region Controller & mechanism

    private final SmartMotorControllerConfig motorConfig;
    private final TalonFXWrapper motor;
    private final FlyWheelConfig flywheelConfig;
    private final FlyWheel flywheel;

    // endregion

    // region Constructor

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

    // region Lifecycle

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
        Logger.recordOutput(
                "Mechanisms/FlywheelIsMoving", Math.abs(flywheelInputs.velocity.in(RPM)) > 10.0);

        // Dashboard-friendly summary: true when the flywheel is spinning and within
        // kVelocityTolerance of its setpoint. Wire to a boolean indicator in Elastic
        // so the operator can confirm the shooter is ready before firing.
        double setpointRPM = flywheelInputs.setpoint.in(RPM);
        double velocityRPM = flywheelInputs.velocity.in(RPM);
        boolean isSpunUp =
                setpointRPM > 10.0 && Math.abs(velocityRPM - setpointRPM) < kVelocityTolerance.in(RPM);
        Logger.recordOutput("Flywheel/IsSpunUp", isSpunUp);

        Robot.batteryLogger.reportCurrentUsage("Flywheel", flywheelInputs.current.in(Amps));
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/Flywheel", flywheelInputs);
        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }

    // endregion

    // region Public API

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

    public Command setToTunedVelocity() {
        // Use the supplier-backed overload so the LoggedTunableNumber is read
        // while the command is active (dashboard edits take effect immediately).
        return this.setVelocity(() -> RPM.of(kTunableFlywheelRPM.get()));
    }

    /**
     * Run the flywheel to the tuned velocity and finish when within tolerance.
     *
     * <p>Prefer this when you need a composition that should progress after the flywheel reaches
     * setpoint (for example: move hood -> runTo flywheel -> feed). This returns the finite YAMS
     * command produced by {@code flywheel.runTo(...)} which completes when the mechanism reaches the
     * target within {@code kVelocityTolerance}.
     */
    public Command runToTunedVelocity() {
        return flywheel.runTo(() -> RPM.of(kTunableFlywheelRPM.get()), kVelocityTolerance);
    }

    public Command clearFlywheel() {
        return flywheel
                .runTo(() -> RPM.of(kTunableFlywheelRPM.get() * -1), kVelocityTolerance)
                .withName("Flywheel_Clear");
    }

    /**
     * One-shot immediate stop: disables closed-loop control and sets motor output to zero, then
     * finishes. Use when you need an imperative, non-blocking stop in a sequence.
     */
    public Command stopNow() {
        return Commands.runOnce(
                        () -> {
                            motor.stopClosedLoopController();
                            motor.setDutyCycle(0);
                        },
                        this)
                .withName("FlywheelStopNow");
    }

    /**
     * Persistent stop command: while scheduled, disables closed-loop control and sets motor output to
     * zero. Use as the default command to keep the mechanism at zero when idle.
     */
    public Command stopHold() {
        return Commands.run(
                        () -> {
                            motor.stopClosedLoopController();
                            motor.setDutyCycle(0);
                        },
                        this)
                .withName("FlywheelStopHold");
    }

    /** Returns the current flywheel velocity in RPM. */
    public double getVelocityRPM() {
        return flywheelInputs.velocity.in(RPM);
    }

    /** Returns the current flywheel setpoint in RPM. */
    public double getSetpointRPM() {
        return flywheelInputs.setpoint.in(RPM);
    }

    /**
     * Returns {@code true} when the flywheel is spinning and within {@code kVelocityTolerance} of its
     * setpoint.
     */
    public boolean atSpeed() {
        double setpointRPM = flywheelInputs.setpoint.in(RPM);
        double velocityRPM = flywheelInputs.velocity.in(RPM);
        return setpointRPM > 10.0 && Math.abs(velocityRPM - setpointRPM) < kVelocityTolerance.in(RPM);
    }

    // endregion
}
