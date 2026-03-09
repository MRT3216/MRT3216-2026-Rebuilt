package frc.robot.constants;

/**
 * Centralized telemetry keys used by AdvantageKit/YAMS logging. Keep keys stable to avoid renames
 * across the codebase.
 */
public final class TelemetryKeys {
    private TelemetryKeys() {}

    // Shooter
    public static final String kFlywheelMotorTelemetry = "FlywheelMotor";
    public static final String kFlywheelMechTelemetry = "FlywheelMech";

    public static final String kSpindexerMotorTelemetry = "SpindexerMotor";
    public static final String kSpindexerMechTelemetry = "SpindexerMech";

    public static final String kKickerMotorTelemetry = "KickerMotor";
    public static final String kKickerMechTelemetry = "KickerMech";

    public static final String kHoodMotorTelemetry = "HoodMotor";
    public static final String kHoodMechTelemetry = "HoodMech";

    public static final String kTurretMotorTelemetry = "TurretMotor";
    public static final String kTurretMechTelemetry = "TurretMech";

    // Intake
    public static final String kIntakeRollersMotorTelemetry = "IntakeRollersMotor";
    public static final String kIntakeRollersMechTelemetry = "IntakeRollersMech";

    public static final String kIntakeArmMotorTelemetry = "IntakeArmMotor";
    public static final String kIntakeArmMechTelemetry = "IntakeArmMech";
}
