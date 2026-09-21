package org.mrt3216.objectvision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;

public final class CameraGeometry {
    private CameraGeometry() {}

    /**
     * Projects a PhotonVision detection onto a horizontal plane at the given height, returning the
     * intersection point in the robot frame.
     *
     * @param robotToCamera transform from robot origin to camera
     * @param yawRadians PhotonVision yaw (left-positive, radians)
     * @param pitchRadians PhotonVision pitch (up-positive, radians)
     * @param targetCenterHeightMeters height of the target plane (meters)
     * @return ground point in robot frame, or empty if the ray does not intersect the plane
     */
    public static Optional<Translation2d> groundPointInRobotFrame(
            Transform3d robotToCamera,
            double yawRadians,
            double pitchRadians,
            double targetCenterHeightMeters) {
        // Camera-frame ray from PhotonVision angles
        Translation3d cameraRay = PhotonAngles.toRay(yawRadians, pitchRadians);

        // Rotate ray into robot frame
        Rotation3d cameraRotation = robotToCamera.getRotation();
        Translation3d robotRay = cameraRay.rotateBy(cameraRotation);

        // Ray must descend toward the target plane
        Translation3d origin = robotToCamera.getTranslation();
        double dz = robotRay.getZ();
        double heightDiff = targetCenterHeightMeters - origin.getZ();

        // If dz is ~zero the ray is horizontal and will never hit the plane.
        // If dz and heightDiff have opposite signs the intersection is behind the camera.
        if (Math.abs(dz) < 1e-9) {
            return Optional.empty();
        }

        double t = heightDiff / dz;
        if (t < 0) {
            return Optional.empty();
        }

        return Optional.of(
                new Translation2d(
                        origin.getX() + t * robotRay.getX(), origin.getY() + t * robotRay.getY()));
    }
}
