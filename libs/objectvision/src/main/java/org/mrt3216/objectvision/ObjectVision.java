package org.mrt3216.objectvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.mrt3216.objectvision.ObjectVisionIO.DetectionObservation;

public class ObjectVision extends SubsystemBase {
    private final TargetConfig config;
    private final Supplier<Pose2d> poseSupplier;
    private final ObjectVisionIO[] ios;
    private final ObjectVisionIOInputsAutoLogged[] inputs;
    private final edu.wpi.first.math.geometry.Transform3d[] robotToCameras;
    private final Alert[] disconnectedAlerts;
    private final RobotPoseHistory poseHistory;
    private final TargetTracker tracker;
    private final ArrayList<Translation2d> fieldPositions = new ArrayList<>();

    public final Trigger hasTarget;

    public ObjectVision(TargetConfig config, Supplier<Pose2d> poseSupplier, CameraSpec... cameras) {
        this.config = config;
        this.poseSupplier = poseSupplier;

        ios = new ObjectVisionIO[cameras.length];
        inputs = new ObjectVisionIOInputsAutoLogged[cameras.length];
        robotToCameras = new edu.wpi.first.math.geometry.Transform3d[cameras.length];
        disconnectedAlerts = new Alert[cameras.length];

        for (int i = 0; i < cameras.length; i++) {
            ios[i] = cameras[i].io();
            inputs[i] = new ObjectVisionIOInputsAutoLogged();
            robotToCameras[i] = cameras[i].robotToCamera();
            disconnectedAlerts[i] =
                    new Alert("Object detection camera " + i + " is disconnected.", AlertType.kWarning);
        }

        poseHistory = new RobotPoseHistory(1.0);
        tracker = new TargetTracker(config);
        hasTarget = new Trigger(() -> !tracker.getTracks().isEmpty());
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        poseHistory.record(now, poseSupplier.get());

        // Update IO and log inputs
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("ObjectVision/Camera" + i, inputs[i]);
            disconnectedAlerts[i].set(!inputs[i].connected);
        }

        // Project detections to field frame
        fieldPositions.clear();
        for (int camIdx = 0; camIdx < ios.length; camIdx++) {
            for (DetectionObservation det : inputs[camIdx].detections) {
                if (det.classId() != config.classId()) {
                    continue;
                }
                if (det.confidence() < config.minConfidence()) {
                    continue;
                }

                Optional<Translation2d> robotPoint =
                        CameraGeometry.groundPointInRobotFrame(
                                robotToCameras[camIdx],
                                det.yawRadians(),
                                det.pitchRadians(),
                                config.targetCenterHeightMeters());
                if (robotPoint.isEmpty()) {
                    continue;
                }

                Optional<Pose2d> historicalPose = poseHistory.sample(det.timestampSeconds());
                if (historicalPose.isEmpty()) {
                    continue;
                }

                // Transform robot-frame point to field frame
                Pose2d pose = historicalPose.get();
                double cos = pose.getRotation().getCos();
                double sin = pose.getRotation().getSin();
                double rx = robotPoint.get().getX();
                double ry = robotPoint.get().getY();
                Translation2d fieldPoint =
                        new Translation2d(pose.getX() + cos * rx - sin * ry, pose.getY() + sin * rx + cos * ry);

                // Range check
                double dx = fieldPoint.getX() - pose.getX();
                double dy = fieldPoint.getY() - pose.getY();
                if (dx * dx + dy * dy > config.maxRangeMeters() * config.maxRangeMeters()) {
                    continue;
                }

                fieldPositions.add(fieldPoint);
            }
        }

        tracker.update(fieldPositions, now);

        // Log tracks
        List<TrackedTarget> tracks = tracker.getTracks();
        Pose2d[] trackPoses = new Pose2d[tracks.size()];
        for (int i = 0; i < tracks.size(); i++) {
            trackPoses[i] =
                    new Pose2d(tracks.get(i).position(), new edu.wpi.first.math.geometry.Rotation2d());
        }
        Logger.recordOutput("ObjectVision/Tracks", trackPoses);
        Logger.recordOutput("ObjectVision/Summary/HasTarget", !tracks.isEmpty());
        Logger.recordOutput("ObjectVision/Summary/TrackCount", tracks.size());
    }

    public List<TrackedTarget> getTracks() {
        return tracker.getTracks();
    }

    public Optional<TrackedTarget> getBest(TargetSelector selector) {
        Optional<TrackedTarget> selected = selector.select(tracker.getTracks(), poseSupplier.get());
        if (selected.isPresent()) {
            Logger.recordOutput(
                    "ObjectVision/Selected",
                    new Pose2d(selected.get().position(), new edu.wpi.first.math.geometry.Rotation2d()));
        }
        return selected;
    }
}
