package org.mrt3216.objectvision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class CameraGeometryTest {
    // Camera mounted at 0.5 m height, pitched 20 degrees down.
    // WPILib Rotation3d: positive pitch = nose down (right-hand rule around Y in NWU).
    private static final double kCameraHeight = 0.5;
    private static final double kCameraPitchRad = Math.toRadians(20.0); // 20 deg down
    private static final Transform3d kRobotToCamera =
            new Transform3d(
                    new Translation3d(0, 0, kCameraHeight), new Rotation3d(0, kCameraPitchRad, 0));

    // FUEL ball center height on carpet
    private static final double kBallCenterHeight = 0.075;

    // Height difference: 0.5 - 0.075 = 0.425 m
    // With 20 deg pitch: distance ahead = 0.425 / tan(20 deg) ≈ 1.168 m
    private static final double kTolerance = 0.005;

    @Test
    void straightAhead_yaw0Pitch0() {
        // Target on optical axis → camera pitch alone determines distance
        Optional<Translation2d> result =
                CameraGeometry.groundPointInRobotFrame(kRobotToCamera, 0, 0, kBallCenterHeight);

        assertTrue(result.isPresent(), "Should intersect the ground plane");
        Translation2d pt = result.get();
        assertEquals(1.168, pt.getX(), kTolerance, "X distance ahead");
        assertEquals(0.0, pt.getY(), kTolerance, "Y should be zero (straight ahead)");
    }

    @Test
    void targetAboveOpticalAxis_pitch10Deg() {
        // Target at pitch +10 deg (above optical axis) → further away on the ground
        // Effective downward angle = 20 - 10 = 10 deg → 0.425 / tan(10 deg) ≈ 2.410 m
        double pitchRad = Math.toRadians(10.0);
        Optional<Translation2d> result =
                CameraGeometry.groundPointInRobotFrame(kRobotToCamera, 0, pitchRad, kBallCenterHeight);

        assertTrue(result.isPresent());
        Translation2d pt = result.get();
        assertEquals(2.410, pt.getX(), kTolerance, "X distance for pitch +10 deg");
        assertEquals(0.0, pt.getY(), kTolerance, "Y should be zero");
    }

    @Test
    void rayAbovePlane_returnsEmpty() {
        // Pitch +25 deg up: combined with 20 deg down camera = net 5 deg up → ray goes up
        double pitchRad = Math.toRadians(25.0);
        Optional<Translation2d> result =
                CameraGeometry.groundPointInRobotFrame(kRobotToCamera, 0, pitchRad, kBallCenterHeight);

        assertTrue(result.isEmpty(), "Ray pointing above the target plane should return empty");
    }

    @Test
    void yawSymmetry() {
        // yaw +a and -a should mirror in Y
        double yawRad = Math.toRadians(15.0);
        Optional<Translation2d> left =
                CameraGeometry.groundPointInRobotFrame(kRobotToCamera, yawRad, 0, kBallCenterHeight);
        Optional<Translation2d> right =
                CameraGeometry.groundPointInRobotFrame(kRobotToCamera, -yawRad, 0, kBallCenterHeight);

        assertTrue(left.isPresent());
        assertTrue(right.isPresent());

        assertEquals(left.get().getX(), right.get().getX(), kTolerance, "X should be equal");
        assertEquals(left.get().getY(), -right.get().getY(), kTolerance, "Y should be mirrored");
        assertTrue(left.get().getY() > 0, "Positive yaw (left) should give positive Y");
    }

    @Test
    void horizontalCamera_returnsEmpty() {
        // Camera with no pitch (horizontal) looking at a target below → ray is horizontal
        Transform3d horizontal =
                new Transform3d(new Translation3d(0, 0, kCameraHeight), new Rotation3d(0, 0, 0));

        Optional<Translation2d> result =
                CameraGeometry.groundPointInRobotFrame(horizontal, 0, 0, kBallCenterHeight);

        assertTrue(result.isEmpty(), "Horizontal ray should not intersect lower plane");
    }

    @Test
    void cameraWithTranslationOffset() {
        // Camera mounted 0.3 m forward and 0.1 m left of robot center
        Transform3d offset =
                new Transform3d(
                        new Translation3d(0.3, 0.1, kCameraHeight), new Rotation3d(0, kCameraPitchRad, 0));

        Optional<Translation2d> result =
                CameraGeometry.groundPointInRobotFrame(offset, 0, 0, kBallCenterHeight);

        assertTrue(result.isPresent());
        Translation2d pt = result.get();
        // The ground point should be shifted by the camera's XY offset
        assertEquals(0.3 + 1.168, pt.getX(), kTolerance, "X includes camera offset");
        assertEquals(0.1, pt.getY(), kTolerance, "Y includes camera offset");
    }
}
