package frc.robot.subsystems.objectvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.mrt3216.objectvision.StaticObjectSource;

/** SIM-only helper that manages simulated ball positions on the field. */
public class ObjectVisionSim {
    public enum Scenario {
        SINGLE_BALL,
        LINE_OF_FIVE,
        CLUSTER
    }

    private static final double kBallHeight = ObjectVisionConstants.kTargetCenterHeightMeters;

    private final StaticObjectSource source = new StaticObjectSource();
    private final SendableChooser<Scenario> scenarioChooser = new SendableChooser<>();
    private final Supplier<Pose2d> poseSupplier;
    private Scenario lastScenario = null;

    public ObjectVisionSim(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        scenarioChooser.setDefaultOption("Single Ball", Scenario.SINGLE_BALL);
        scenarioChooser.addOption("Line of Five", Scenario.LINE_OF_FIVE);
        scenarioChooser.addOption("Cluster", Scenario.CLUSTER);
        SmartDashboard.putData("ObjectVisionSim/Scenario", scenarioChooser);
    }

    /** Call periodically to check for scenario changes and log truth. */
    public void update() {
        Scenario selected = scenarioChooser.getSelected();
        if (selected != lastScenario) {
            lastScenario = selected;
            loadScenario(poseSupplier.get());
        }
        logTruth();
    }

    /** Place balls relative to the robot's current pose. */
    public void loadScenario(Pose2d robotPose) {
        source.clear();
        double cos = robotPose.getRotation().getCos();
        double sin = robotPose.getRotation().getSin();
        double rx = robotPose.getX();
        double ry = robotPose.getY();

        switch (lastScenario != null ? lastScenario : Scenario.SINGLE_BALL) {
            case SINGLE_BALL:
                // One ball 2 m ahead of the robot
                addRelative(rx, ry, cos, sin, 2.0, 0.0);
                break;

            case LINE_OF_FIVE:
                // Five balls in a line, 1.5 m apart, starting 1.5 m ahead
                for (int i = 0; i < 5; i++) {
                    addRelative(rx, ry, cos, sin, 1.5 + i * 1.5, 0.0);
                }
                break;

            case CLUSTER:
                // Tight cluster of 4 balls, 3 m ahead, ~0.3 m apart
                addRelative(rx, ry, cos, sin, 3.0, 0.0);
                addRelative(rx, ry, cos, sin, 3.0, 0.3);
                addRelative(rx, ry, cos, sin, 3.3, 0.0);
                addRelative(rx, ry, cos, sin, 3.3, 0.3);
                break;
        }
    }

    private void addRelative(double rx, double ry, double cos, double sin, double fwd, double left) {
        double fieldX = rx + cos * fwd - sin * left;
        double fieldY = ry + sin * fwd + cos * left;
        source.add(new Translation3d(fieldX, fieldY, kBallHeight));
    }

    /** Remove balls near the intake when intake is active. */
    public void removeNearIntake(Pose2d robot, boolean intakeActive) {
        if (intakeActive) {
            // Intake rectangle: 0.5 m ahead, 0.3 m wide
            source.removeWithin(robot, 0.5, 0.3);
        }
    }

    public Supplier<List<Translation3d>> getBallSupplier() {
        return source::getPositions;
    }

    private void logTruth() {
        List<Translation3d> balls = source.getPositions();
        Pose2d[] truthPoses = new Pose2d[balls.size()];
        for (int i = 0; i < balls.size(); i++) {
            truthPoses[i] = new Pose2d(balls.get(i).getX(), balls.get(i).getY(), new Rotation2d());
        }
        Logger.recordOutput("ObjectVisionSim/Truth", truthPoses);
    }
}
