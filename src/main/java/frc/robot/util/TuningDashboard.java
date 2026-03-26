package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.util.shooter.HybridTurretUtil;
import frc.robot.util.shooter.ShooterModel;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Creates a "Tuning" Shuffleboard tab with widgets for shooter LUT calibration, turret
 * visualization, and general mechanism readouts. Only initialized when {@code Constants.tuningMode}
 * is {@code true}, so the tab never appears on the Elastic dashboard during competition.
 *
 * <p>Call {@link #initialize} once from {@code RobotContainer}'s constructor.
 */
public final class TuningDashboard {
    private TuningDashboard() {}

    // Mechanism2d instances (kept as fields so periodic() can update them)
    private static Mechanism2d turretMech;
    private static MechanismLigament2d turretArmLigament;
    private static MechanismLigament2d turretSetpointLigament;
    private static MechanismLigament2d hoodLigament;

    // Turret dead-zone markers
    private static MechanismLigament2d deadZoneCW;
    private static MechanismLigament2d deadZoneCCW;

    // Soft limit markers
    private static MechanismLigament2d softLimitCW;
    private static MechanismLigament2d softLimitCCW;

    // Supplier references for periodic updates
    private static Supplier<Double> turretPositionDeg;
    private static Supplier<Double> turretSetpointDeg;
    private static Supplier<Double> hoodPositionDeg;

    /**
     * Creates the "Tuning" Shuffleboard tab and populates it with widgets.
     *
     * @param drive the drive subsystem (for pose-based distance calculations)
     * @param turret the turret subsystem
     * @param hood the hood subsystem
     * @param flywheel the flywheel subsystem
     */
    public static void initialize(
            Drive drive, TurretSubsystem turret, HoodSubsystem hood, FlywheelSubsystem flywheel) {

        ShuffleboardTab tab = Shuffleboard.getTab("Tuning");

        // ── Turret Mechanism2d visualization ──────────────────────────────
        // A top-down view of the turret. The turret arm rotates to show the
        // current position (cyan) and setpoint (ghost green). The hood is
        // shown as a shorter ligament extending from the turret arm tip.
        turretMech = new Mechanism2d(3, 3);
        MechanismRoot2d root = turretMech.getRoot("Turret", 1.5, 1.5);

        // Current position arm (thick cyan)
        turretArmLigament =
                root.append(new MechanismLigament2d("TurretArm", 1.0, 0, 6, new Color8Bit(0, 255, 255)));

        // Setpoint ghost arm (thin green, overlaid)
        turretSetpointLigament =
                root.append(new MechanismLigament2d("TurretSetpoint", 1.0, 0, 2, new Color8Bit(0, 255, 0)));

        // Hood ligament extending from turret arm tip (orange)
        hoodLigament =
                turretArmLigament.append(
                        new MechanismLigament2d("Hood", 0.3, 0, 4, new Color8Bit(255, 165, 0)));

        // Hard-limit markers (thin red) — the physical stops at kHardLimitMax / kHardLimitMin.
        // These are static reference lines.
        deadZoneCW =
                root.append(new MechanismLigament2d(
                        "DeadZoneCW", 1.2, TurretConstants.kHardLimitMax.in(Degrees), 1, new Color8Bit(255, 0, 0)));
        deadZoneCCW =
                root.append(new MechanismLigament2d(
                        "DeadZoneCCW", 1.2, TurretConstants.kHardLimitMin.in(Degrees), 1, new Color8Bit(255, 0, 0)));

        // Soft limit markers (thin yellow)
        softLimitCW =
                root.append(
                        new MechanismLigament2d(
                                "SoftLimitCW",
                                1.3,
                                TurretConstants.kSoftLimitMax.in(Degrees),
                                1,
                                new Color8Bit(255, 255, 0)));
        softLimitCCW =
                root.append(
                        new MechanismLigament2d(
                                "SoftLimitCCW",
                                1.3,
                                TurretConstants.kSoftLimitMin.in(Degrees),
                                1,
                                new Color8Bit(255, 255, 0)));

        // Store suppliers for periodic updates
        turretPositionDeg = () -> turret.getPosition().in(Degrees);
        turretSetpointDeg = () -> turret.getTarget().in(Degrees);
        hoodPositionDeg = () -> hood.getPosition().in(Degrees);

        // Place the Mechanism2d on the tab
        tab.add("Turret Viz", turretMech).withPosition(0, 0).withSize(4, 4);

        // ── Turret readouts ──────────────────────────────────────────────
        ShuffleboardLayout turretLayout =
                tab.getLayout("Turret", BuiltInLayouts.kList)
                        .withPosition(4, 0)
                        .withSize(2, 4)
                        .withProperties(Map.of("Label position", "LEFT"));

        turretLayout.addDouble("Position (°)", () -> turret.getPosition().in(Degrees));
        turretLayout.addDouble("Setpoint (°)", () -> turret.getTarget().in(Degrees));
        turretLayout.addDouble(
                "Error (°)", () -> turret.getTarget().in(Degrees) - turret.getPosition().in(Degrees));
        turretLayout.addBoolean(
                "At Setpoint",
                () -> Math.abs(turret.getTarget().in(Degrees) - turret.getPosition().in(Degrees)) < 1.0);

        // ── Hood readouts ────────────────────────────────────────────────
        ShuffleboardLayout hoodLayout =
                tab.getLayout("Hood", BuiltInLayouts.kList)
                        .withPosition(6, 0)
                        .withSize(2, 3)
                        .withProperties(Map.of("Label position", "LEFT"));

        hoodLayout.addDouble("Position (°)", () -> hood.getPosition().in(Degrees));
        hoodLayout.addDouble("Setpoint (°)", () -> hood.getTarget().in(Degrees));
        hoodLayout.addDouble(
                "Error (°)", () -> hood.getTarget().in(Degrees) - hood.getPosition().in(Degrees));

        // ── Flywheel readouts ────────────────────────────────────────────
        ShuffleboardLayout flywheelLayout =
                tab.getLayout("Flywheel", BuiltInLayouts.kList)
                        .withPosition(6, 3)
                        .withSize(2, 3)
                        .withProperties(Map.of("Label position", "LEFT"));

        flywheelLayout.addDouble("Velocity (RPM)", flywheel::getVelocityRPM);
        flywheelLayout.addDouble("Setpoint (RPM)", flywheel::getSetpointRPM);
        flywheelLayout.addBoolean("At Speed", flywheel::atSpeed);

        // ── Distance & LUT calibration data ──────────────────────────────
        // These values are the core data needed to build/update the shooter
        // lookup table. Distance is computed from the turret center to the
        // hub center via HybridTurretUtil.turretOrigin(), matching
        // ShooterSystem.testShoot exactly.
        Supplier<Distance> hubDistSupplier =
                () -> {
                    Translation3d hub = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);
                    var tOrigin = HybridTurretUtil.turretOrigin(drive.getPose());
                    double dx = hub.toTranslation2d().getX() - tOrigin.getX();
                    double dy = hub.toTranslation2d().getY() - tOrigin.getY();
                    return edu.wpi.first.units.Units.Meters.of(Math.hypot(dx, dy));
                };

        ShuffleboardLayout lutLayout =
                tab.getLayout("LUT Calibration", BuiltInLayouts.kList)
                        .withPosition(8, 0)
                        .withSize(2, 5)
                        .withProperties(Map.of("Label position", "LEFT"));

        lutLayout.addDouble(
                "Distance (m)", () -> hubDistSupplier.get().in(edu.wpi.first.units.Units.Meters));
        lutLayout.addDouble(
                "Distance (in)",
                () -> hubDistSupplier.get().in(edu.wpi.first.units.Units.Meters) * 39.3701);
        lutLayout.addDouble(
                "Model RPM", () -> ShooterModel.flywheelRPMForDistance(hubDistSupplier.get()));
        lutLayout.addDouble("Hood Angle (°)", () -> hood.getPosition().in(Degrees));
        lutLayout.addDouble("Turret Angle (°)", () -> turret.getPosition().in(Degrees));
        lutLayout.addString(
                "LUT Row",
                () -> {
                    double distM = hubDistSupplier.get().in(edu.wpi.first.units.Units.Meters);
                    double hoodDeg = hood.getPosition().in(Degrees);
                    return String.format("%.2f m, %.1f°", distM, hoodDeg);
                });

        // ── Shoot Mode & Status ──────────────────────────────────────────
        ShuffleboardLayout statusLayout =
                tab.getLayout("Status", BuiltInLayouts.kList)
                        .withPosition(4, 4)
                        .withSize(2, 2)
                        .withProperties(Map.of("Label position", "LEFT"));

        statusLayout.addBoolean("Flywheel Ready", flywheel::atSpeed);
        statusLayout.addBoolean(
                "Turret On Target",
                () -> Math.abs(turret.getTarget().in(Degrees) - turret.getPosition().in(Degrees)) < 1.0);
        statusLayout.addBoolean(
                "Hood On Target",
                () -> Math.abs(hood.getTarget().in(Degrees) - hood.getPosition().in(Degrees)) < 0.5);
    }

    /**
     * Update the Mechanism2d visualization with current subsystem state. Call this from a periodic
     * context (e.g., a default command or Robot.periodic) when tuning mode is active.
     */
    public static void periodic() {
        if (turretArmLigament == null) return; // not initialized

        // Update turret arm angle — Mechanism2d uses CCW-positive with 0° = right,
        // which matches our turret convention (0° = robot forward ≈ right on screen).
        turretArmLigament.setAngle(turretPositionDeg.get());
        turretSetpointLigament.setAngle(turretSetpointDeg.get());

        // Hood extends from the turret arm tip, offset by the hood angle
        hoodLigament.setAngle(hoodPositionDeg.get());
    }
}
