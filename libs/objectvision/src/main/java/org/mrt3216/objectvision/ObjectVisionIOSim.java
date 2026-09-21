package org.mrt3216.objectvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;

public class ObjectVisionIOSim implements ObjectVisionIO {
    private final Supplier<Pose2d> truthPoseSupplier;
    private final Supplier<List<Translation3d>> ballSupplier;
    private final Transform3d robotToCamera;
    private final SimCameraModel model;
    private final Random rng;

    // Inverse camera rotation for transforming world points into camera frame
    private final Rotation3d cameraRotationInverse;
    private final Translation3d cameraTranslation;

    // Frame-rate limiting
    private double lastFrameTimestamp = 0.0;

    // Latency buffer
    private final ArrayDeque<LatencyEntry> latencyBuffer = new ArrayDeque<>();

    // Preallocated scratch list
    private final ArrayList<DetectionObservation> scratchDetections = new ArrayList<>();

    // Config-derived constants
    private final int classId;
    private final double halfHFov;
    private final double halfVFov;
    private final double maxRangeSq;

    public ObjectVisionIOSim(
            Supplier<Pose2d> truthPoseSupplier,
            Supplier<List<Translation3d>> ballSupplier,
            Transform3d robotToCamera,
            SimCameraModel model,
            int classId) {
        this.truthPoseSupplier = truthPoseSupplier;
        this.ballSupplier = ballSupplier;
        this.robotToCamera = robotToCamera;
        this.model = model;
        this.classId = classId;
        this.rng = new Random(42);

        this.cameraRotationInverse = robotToCamera.getRotation().unaryMinus();
        this.cameraTranslation = robotToCamera.getTranslation();

        this.halfHFov = model.hFovRadians() / 2.0;
        this.halfVFov = model.vFovRadians() / 2.0;
        this.maxRangeSq = model.maxRangeMeters() * model.maxRangeMeters();
    }

    @Override
    public void updateInputs(ObjectVisionIOInputs inputs) {
        inputs.connected = true;
        double now = Timer.getFPGATimestamp();

        // Frame-rate limiting
        double framePeriod = 1.0 / model.frameRateHz();
        if (now - lastFrameTimestamp >= framePeriod) {
            lastFrameTimestamp = now;
            generateDetections(now);
        }

        // Pop detections that have exceeded latency from the buffer
        inputs.detections = popReady(now);
    }

    private void generateDetections(double timestamp) {
        scratchDetections.clear();
        Pose2d robotPose = truthPoseSupplier.get();
        List<Translation3d> balls = ballSupplier.get();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double cos = robotPose.getRotation().getCos();
        double sin = robotPose.getRotation().getSin();

        for (int i = 0; i < balls.size(); i++) {
            Translation3d ball = balls.get(i);

            // 1. Transform ball from field frame to robot frame
            double dx = ball.getX() - robotX;
            double dy = ball.getY() - robotY;
            double robotFrameX = cos * dx + sin * dy;
            double robotFrameY = -sin * dx + cos * dy;
            double robotFrameZ = ball.getZ();

            // 2. Transform from robot frame to camera frame
            double camRelX = robotFrameX - cameraTranslation.getX();
            double camRelY = robotFrameY - cameraTranslation.getY();
            double camRelZ = robotFrameZ - cameraTranslation.getZ();
            Translation3d camPoint =
                    new Translation3d(camRelX, camRelY, camRelZ).rotateBy(cameraRotationInverse);

            // 3. Range check (in robot frame, from camera origin)
            double rangeSq = camRelX * camRelX + camRelY * camRelY + camRelZ * camRelZ;
            if (rangeSq > maxRangeSq || rangeSq < 1e-6) {
                continue;
            }

            // 4. Must be in front of camera (positive X in camera NWU frame)
            if (camPoint.getX() <= 0) {
                continue;
            }

            // 5. Compute yaw and pitch (same convention as PhotonAngles)
            double yaw = Math.atan2(camPoint.getY(), camPoint.getX());
            double pitch = Math.atan2(camPoint.getZ(), Math.hypot(camPoint.getX(), camPoint.getY()));

            // 6. FOV check
            if (Math.abs(yaw) > halfHFov || Math.abs(pitch) > halfVFov) {
                continue;
            }

            // 7. Dropout
            if (rng.nextDouble() < model.dropoutProbability()) {
                continue;
            }

            // 8. Add noise
            yaw += rng.nextGaussian() * model.yawNoiseStdDevRadians();
            pitch += rng.nextGaussian() * model.pitchNoiseStdDevRadians();

            scratchDetections.add(
                    new DetectionObservation(
                            classId,
                            0.8 + rng.nextDouble() * 0.2, // confidence 0.8–1.0
                            yaw,
                            pitch,
                            0.0, // areaPercent not modeled
                            timestamp));
        }

        // Push to latency buffer
        DetectionObservation[] frame = new DetectionObservation[scratchDetections.size()];
        for (int i = 0; i < scratchDetections.size(); i++) {
            frame[i] = scratchDetections.get(i);
        }
        latencyBuffer.addLast(new LatencyEntry(timestamp, frame));
    }

    private DetectionObservation[] popReady(double now) {
        double readyTime = now - model.latencySeconds();
        // Collect all entries that are ready
        ArrayList<DetectionObservation> ready = null;
        while (!latencyBuffer.isEmpty() && latencyBuffer.peekFirst().timestamp <= readyTime) {
            LatencyEntry entry = latencyBuffer.pollFirst();
            if (entry.detections.length > 0) {
                if (ready == null) {
                    ready = new ArrayList<>();
                }
                for (DetectionObservation det : entry.detections) {
                    ready.add(det);
                }
            }
        }
        if (ready == null) {
            return new DetectionObservation[0];
        }
        return ready.toArray(new DetectionObservation[0]);
    }

    private record LatencyEntry(double timestamp, DetectionObservation[] detections) {}
}
