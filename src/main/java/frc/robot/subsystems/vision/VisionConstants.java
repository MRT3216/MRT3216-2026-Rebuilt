// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Vision pipeline constants for PhotonVision AprilTag pose estimation.
 *
 * <p>Camera resolution and exposure settings are configured in the PhotonVision web UI ({@code
 * http://photonvision.local:5800}), not in code. See {@code docs/TuningGuide.md} § "PhotonVision
 * Camera Tuning" for resolution/FPS tradeoffs and recommended exposure/gain values.
 *
 * <p><b>Reference: 6328 (2026)</b> — color cameras at 1280×960 (exposure=4500µs, gain=5) and mono
 * cameras at 1800×1200 (exposure=1800µs, gain=15). Higher resolution extends detection range but
 * reduces frame rate.
 */
public final class VisionConstants {
    private VisionConstants() {}

    // AprilTag layout — final so the JVM initializes it exactly once at class-load
    // time
    public static final AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static final String cameraLeftName = "Left_Cam";
    public static final String cameraRightName = "Right_Cam";
    public static final String cameraFrontName = "Front_Cam";
    public static final String cameraBackName = "Rear_Cam";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    // NOTE: Rotation3d args mix Degrees and Radians — this is intentional (pitch is
    // in radians).
    public static final Transform3d robotToCameraLeft =
            new Transform3d(
                    Inches.of(-11.791238),
                    Inches.of(12.876205),
                    Inches.of(14.154937),
                    new Rotation3d(Degrees.of(0.0), Radians.of(-0.2), Degrees.of(75)));

    public static final Transform3d robotToCameraRight =
            new Transform3d(
                    Inches.of(-11.427808),
                    Inches.of(-12.304215),
                    Inches.of(14.175991),
                    new Rotation3d(Degrees.of(0), Radians.of(-0.4), Degrees.of(285)));

    public static final Transform3d robotToCameraFront =
            new Transform3d(
                    Inches.of(-11.234075),
                    Inches.of(-10.772856),
                    Inches.of(20.375546),
                    new Rotation3d(Degrees.of(0), Radians.of(-0.4), Degrees.of(15)));

    public static final Transform3d robotToCameraBack =
            new Transform3d(
                    Inches.of(-12.865422),
                    Inches.of(11.805673),
                    Inches.of(16.151008),
                    new Rotation3d(Degrees.of(0), Radians.of(-0.2), Degrees.of(195)));

    // Basic filtering thresholds
    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    //
    // Reference values from other teams (2026 season):
    // BroncBotz 3481: VecBuilder.fill(0.05, 0.05, 0.022)
    // 6328: Uses per-camera trust factors, similar scale
    //
    // Lower values = trust vision more. Increase if the robot "jumps" on the field
    // view.
    public static final double linearStdDevBaseline = 0.02; // Meters
    public static final double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] cameraStdDevFactors =
            new double[] {
                1.0, // Camera 0 (Left)
                1.0, // Camera 1 (Right)
                1.0, // Camera 2 (Front)
                1.0 // Camera 3 (Back)
            };

    // Multipliers to apply for MegaTag 2 observations
    public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static final double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available
}
