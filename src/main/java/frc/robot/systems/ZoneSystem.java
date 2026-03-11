package frc.robot.systems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.constants.ShooterConstants.HoodConstants;
import frc.robot.util.geometry.Zones;
import java.util.function.Supplier;

public class ZoneSystem {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;

    public ZoneSystem(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
    }

    /** Returns a trigger that is active when the robot is inside any trench zone. */
    public Trigger inTrench() {
        return Zones.TRENCH_ZONES.contains(poseSupplier);
    }

    /** Returns a trigger that is active when the robot is predicted to enter a trench zone. */
    public Trigger willEnterTrench() {
        return Zones.TRENCH_ZONES.willContain(
                poseSupplier, speedsSupplier, HoodConstants.kDuckDuration);
    }

    /** Returns a trigger that is active when the robot is inside any bump zone. */
    public Trigger inBump() {
        return Zones.BUMP_ZONES.willContain(poseSupplier, speedsSupplier, HoodConstants.kDuckDuration);
    }

    /** Returns a trigger that is active when the robot is predicted to enter a bump zone. */
    public Trigger willEnterBump() {
        return Zones.BUMP_ZONES.willContain(
                poseSupplier, speedsSupplier, DriveConstants.kRotateSnapDuration);
    }

    /** Returns a trigger that is active when the robot is inside an alliance zone. */
    public Trigger inAllianceZone() {
        return Zones.ALLIANCE_ZONES.contains(poseSupplier);
    }

    /** Returns a trigger that is active when the robot is inside the neutral zone. */
    public Trigger inNeutralZone() {
        return Zones.NEUTRAL_ZONES.contains(poseSupplier);
    }

    // public ZoneSystem(ShooterSystem shooterSystem, IntakeSystem intakeSystem, DriveCommands
    // driveCommands) {
    // }

    /** Periodic logging of zone boundaries to AdvantageScope. */
    public void log() {
        frc.robot.util.geometry.Zones.logAllZones();
    }
}
