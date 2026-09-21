package org.mrt3216.objectvision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class ObjectVisionIOSimTest {
    // Camera: 0.5 m high, pitched 20 deg down, centered on robot
    private static final Transform3d kRobotToCamera =
            new Transform3d(new Translation3d(0.3, 0.0, 0.5), new Rotation3d(0, Math.toRadians(20.0), 0));
    private static final double kBallHeight = 0.075;
    private static final int kClassId = 0;

    /** No-noise sim: project a ball, recover with CameraGeometry, expect near-zero error. */
    @Test
    void roundTrip_noNoise() {
        // Robot at origin facing +X
        Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());
        // Ball 3 m ahead on the ground
        Translation3d ball = new Translation3d(3.0, 0.0, kBallHeight);

        // Zero noise, zero dropout, zero latency, high frame rate
        SimCameraModel model =
                new SimCameraModel(
                        Math.toRadians(70),
                        Math.toRadians(43),
                        10.0, // maxRange
                        0.0, // no yaw noise
                        0.0, // no pitch noise
                        0.0, // no dropout
                        0.0, // no latency
                        1000.0); // high frame rate so we always get a frame

        ObjectVisionIOSim sim =
                new ObjectVisionIOSim(
                        () -> robotPose, () -> List.of(ball), kRobotToCamera, model, kClassId);

        ObjectVisionIO.ObjectVisionIOInputs inputs = new ObjectVisionIO.ObjectVisionIOInputs();
        sim.updateInputs(inputs);

        assertEquals(1, inputs.detections.length, "Should detect exactly one ball");

        ObjectVisionIO.DetectionObservation det = inputs.detections[0];
        assertEquals(kClassId, det.classId());

        // Recover ground point using CameraGeometry
        Optional<Translation2d> recovered =
                CameraGeometry.groundPointInRobotFrame(
                        kRobotToCamera, det.yawRadians(), det.pitchRadians(), kBallHeight);

        assertTrue(recovered.isPresent(), "Should recover a ground point");

        // Transform recovered robot-frame point to field frame
        Translation2d rp = recovered.get();
        double fieldX = robotPose.getX() + rp.getX();
        double fieldY = robotPose.getY() + rp.getY();

        double error = Math.hypot(fieldX - ball.getX(), fieldY - ball.getY());
        assertTrue(
                error < 0.01, "Round-trip error should be < 1 cm without noise, got " + error + " m");
    }

    /**
     * With noise: average error should be bounded. At 3 m range the effective pitch angle is only
     * ~8°, so 1° pitch noise causes large range errors (d/tan(θ) sensitivity). We use a generous 0.50
     * m tolerance and average over multiple frames from a single sim (RNG advances each frame).
     */
    @Test
    void roundTrip_withNoise() {
        Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());
        Translation3d ball = new Translation3d(3.0, 0.0, kBallHeight);

        SimCameraModel model =
                new SimCameraModel(
                        Math.toRadians(70),
                        Math.toRadians(43),
                        10.0,
                        Math.toRadians(1.0), // 1 deg yaw noise
                        Math.toRadians(1.0), // 1 deg pitch noise
                        0.0, // no dropout for this test
                        0.0, // no latency
                        1000.0);

        ObjectVisionIOSim sim =
                new ObjectVisionIOSim(
                        () -> robotPose, () -> List.of(ball), kRobotToCamera, model, kClassId);

        double totalError = 0;
        int detected = 0;

        for (int t = 0; t < 50; t++) {
            ObjectVisionIO.ObjectVisionIOInputs inputs = new ObjectVisionIO.ObjectVisionIOInputs();
            sim.updateInputs(inputs);

            if (inputs.detections.length == 0) {
                continue;
            }
            detected++;

            ObjectVisionIO.DetectionObservation det = inputs.detections[0];
            Optional<Translation2d> recovered =
                    CameraGeometry.groundPointInRobotFrame(
                            kRobotToCamera, det.yawRadians(), det.pitchRadians(), kBallHeight);

            if (recovered.isEmpty()) {
                continue;
            }

            Translation2d rp = recovered.get();
            double fieldX = robotPose.getX() + rp.getX();
            double fieldY = robotPose.getY() + rp.getY();
            totalError += Math.hypot(fieldX - ball.getX(), fieldY - ball.getY());
        }

        assertTrue(detected > 0, "Should detect ball in at least some trials");
        double avgError = totalError / detected;
        assertTrue(
                avgError < 0.50,
                "Average round-trip error at 3 m with 1 deg noise should be < 0.50 m, got "
                        + avgError
                        + " m");
    }

    /** Ball outside FOV should not be detected. */
    @Test
    void ballOutsideFOV_notDetected() {
        Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());
        // Ball far to the side — outside 70 deg hFOV
        Translation3d ball = new Translation3d(1.0, 5.0, kBallHeight);

        SimCameraModel model =
                new SimCameraModel(
                        Math.toRadians(70), Math.toRadians(43), 10.0, 0.0, 0.0, 0.0, 0.0, 1000.0);

        ObjectVisionIOSim sim =
                new ObjectVisionIOSim(
                        () -> robotPose, () -> List.of(ball), kRobotToCamera, model, kClassId);

        ObjectVisionIO.ObjectVisionIOInputs inputs = new ObjectVisionIO.ObjectVisionIOInputs();
        sim.updateInputs(inputs);

        assertEquals(0, inputs.detections.length, "Ball outside FOV should not be detected");
    }

    /** Ball behind camera should not be detected. */
    @Test
    void ballBehindCamera_notDetected() {
        Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());
        Translation3d ball = new Translation3d(-2.0, 0.0, kBallHeight);

        SimCameraModel model =
                new SimCameraModel(
                        Math.toRadians(70), Math.toRadians(43), 10.0, 0.0, 0.0, 0.0, 0.0, 1000.0);

        ObjectVisionIOSim sim =
                new ObjectVisionIOSim(
                        () -> robotPose, () -> List.of(ball), kRobotToCamera, model, kClassId);

        ObjectVisionIO.ObjectVisionIOInputs inputs = new ObjectVisionIO.ObjectVisionIOInputs();
        sim.updateInputs(inputs);

        assertEquals(0, inputs.detections.length, "Ball behind camera should not be detected");
    }
}
