package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kD;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kD_sim;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kEncoderZeroOffset;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kGearing;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kHardLimitMax;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kHardLimitMin;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kI;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kI_sim;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kLength;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kMass;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kMaxAccel;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kMaxVelocity;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kMotorInverted;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kP;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kP_sim;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kSoftLimitMax;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kSoftLimitMin;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kStatorCurrentLimit;
import static frc.robot.subsystems.intake.IntakeConstants.Pivot.kTolerance;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * AdvantageKit-ready Intake Pivot Subsystem for MRT 3216.
 *
 * <p>This subsystem manages a single-Kraken intake arm pivot using the YAMS library and Phoenix 6.
 * It utilizes an IO-layer abstraction for full log replay capabilities, ensuring that hardware
 * states (Inputs) are separated from software commands (Outputs).
 */
public class IntakePivotSubsystem extends SubsystemBase {
    private static final String kIntakeArmMotorTelemetry = "IntakeArmMotor";
    private static final String kIntakeArmMechTelemetry = "IntakeArmMech";

    // region Inputs & telemetry

    /**
     * IO inputs for the Intake Pivot. AutoLogged to provide synchronized data for AdvantageScope and
     * log replay.
     */
    @AutoLog
    public static class IntakePivotInputs {
        /** Actual angle of the intake arm. */
        public Angle angle = Degrees.of(0);
        /** Actual angular velocity of the intake arm. */
        public AngularVelocity velocity = DegreesPerSecond.of(0);
        /** Current target angle requested from the motor controller. */
        public Angle setpoint = Degrees.of(0);
        /** Applied voltage across the motor. */
        public Voltage volts = Volts.of(0);
        /** Stator current draw of the motor. */
        public Current current = Amps.of(0);
    }

    private final IntakePivotInputsAutoLogged intakePivotInputs = new IntakePivotInputsAutoLogged();

    // endregion

    // region Hardware

    /* Hardware controllers (left master, right follower) */
    private final SparkFlex leftPivotMotor =
            new SparkFlex(RobotMap.Intake.Pivot.kLeftMotorId, SparkFlex.MotorType.kBrushless);
    private final SparkFlex rightPivotMotor =
            new SparkFlex(RobotMap.Intake.Pivot.kRightMotorId, SparkFlex.MotorType.kBrushless);

    // endregion

    // region Controller & mechanism

    /* Configuration for the Smart Motor Controller (SMC) */
    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController smartMotor;

    /* High-level mechanism configuration */
    private final ArmConfig intakePivotConfig;

    private final Arm intakePivot;

    // endregion

    // region Constructor

    /** Initializes the subsystem, sets signal update frequencies, and optimizes CAN utilization. */
    public IntakePivotSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(kP, kI, kD, kMaxVelocity, kMaxAccel)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim, kMaxVelocity, kMaxAccel)
                        // .withFeedforward(armFeedforward())
                        // .withSimFeedforward(armFeedforwardSim())
                        .withTelemetry(kIntakeArmMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(kGearing)
                        .withMotorInverted(kMotorInverted)
                        .withIdleMode(MotorMode.BRAKE)
                        .withStatorCurrentLimit(kStatorCurrentLimit)
                        .withVoltageCompensation(Volts.of(12))
                        .withOpenLoopRampRate(Seconds.of(0.1))
                        .withExternalEncoder(leftPivotMotor.getAbsoluteEncoder())
                        .withExternalEncoderInverted(false)
                        .withUseExternalFeedbackEncoder(true)
                        .withExternalEncoderZeroOffset(kEncoderZeroOffset)
                        .withExternalEncoderGearing(new MechanismGearing(GearBox.fromStages("1:1")))
                        .withFollowers(Pair.of(rightPivotMotor, true));

        smartMotor = new SparkWrapper(leftPivotMotor, DCMotor.getNeoVortex(2), motorConfig);

        intakePivotConfig =
                new ArmConfig(smartMotor)
                        .withMass(kMass)
                        .withLength(kLength)
                        .withTelemetry(kIntakeArmMechTelemetry, Constants.telemetryVerbosity())
                        .withSoftLimits(kSoftLimitMin, kSoftLimitMax)
                        .withHardLimit(kHardLimitMin, kHardLimitMax);

        intakePivot = new Arm(intakePivotConfig);
    }

    // endregion

    // region Lifecycle

    /**
     * Updates the AdvantageKit "inputs" by reading hardware state. Provides synchronized telemetry
     * for log replay.
     */
    private void updateInputs() {
        intakePivotInputs.angle = intakePivot.getAngle();
        intakePivotInputs.velocity = smartMotor.getMechanismVelocity();
        intakePivotInputs.volts = smartMotor.getVoltage();
        intakePivotInputs.current = smartMotor.getStatorCurrent();
        intakePivotInputs.setpoint = smartMotor.getMechanismPositionSetpoint().orElse(Degrees.of(0));
        Logger.recordOutput(
                "Mechanisms/IntakePivotIsMoving",
                Math.abs(intakePivotInputs.velocity.in(DegreesPerSecond)) > 2.0);

        Robot.batteryLogger.reportCurrentUsage("IntakePivot", intakePivotInputs.current.in(Amps));
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Intake/Pivot", intakePivotInputs);
        intakePivot.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        intakePivot.simIterate();
    }

    // endregion

    // region Public API

    /**
     * Sets the target angle for the intake arm.
     *
     * @param angle The target Angle.
     * @return A command to set and maintain the requested angle.
     */
    public Command setAngle(Angle angle) {
        return intakePivot.run(angle);
    }

    /**
     * Move the intake arm to the requested angle and finish when within tolerance.
     *
     * @param angle target arm angle
     * @return a command that completes when the arm reaches the target
     */
    public Command setAngleAndStop(Angle angle) {
        return intakePivot.runTo(angle, kTolerance);
    }

    /**
     * Sets the duty cycle (percent output) for the intake arm.
     *
     * @param dutyCycle The output percentage (-1.0 to 1.0).
     * @return A command to run the intake arm at the specified duty cycle.
     */
    public Command set(double dutyCycle) {
        return intakePivot.set(dutyCycle);
    }

    /**
     * Sets a fixed voltage on the intake arm. Unlike duty-cycle control, this delivers the same
     * torque regardless of battery voltage, making behaviour consistent across matches.
     *
     * @param volts The voltage to apply (positive = pull in / retract, negative = push out / deploy).
     * @return A command to run the intake arm at the specified voltage.
     */
    public Command setVoltage(Voltage volts) {
        return intakePivot.setVoltage(volts);
    }

    /** Run a YAMS SysId routine for feedforward characterization. */
    public Command sysId() {
        return intakePivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
    }

    // endregion
}
