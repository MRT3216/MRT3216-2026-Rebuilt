package org.mrt3216.objectvision;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Single source of truth for converting PhotonVision yaw/pitch angles to a camera-frame ray.
 *
 * <p>PhotonVision conventions (verified against v2026.3.2):
 *
 * <ul>
 *   <li>Yaw: left = positive, right = negative
 *   <li>Pitch: up = positive, down = negative
 * </ul>
 *
 * <p>Camera frame is WPILib NWU: X forward, Y left, Z up.
 */
public final class PhotonAngles {
    private PhotonAngles() {}

    /**
     * Converts PhotonVision yaw and pitch (radians) to a unit ray in the camera frame.
     *
     * @param yawRadians yaw angle in radians (left-positive)
     * @param pitchRadians pitch angle in radians (up-positive)
     * @return unit ray in camera frame (NWU)
     */
    public static Translation3d toRay(double yawRadians, double pitchRadians) {
        double cosPitch = Math.cos(pitchRadians);
        return new Translation3d(
                cosPitch * Math.cos(yawRadians), cosPitch * Math.sin(yawRadians), Math.sin(pitchRadians));
    }
}
