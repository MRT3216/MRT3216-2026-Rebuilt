// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String cameraLeftName = "camera_left";
    public static String cameraRightName = "camera_right";
    public static String cameraFrontName = "camera_front";
    public static String cameraBackName = "camera_back";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCameraLeft =
            new Transform3d(
                    Units.inchesToMeters(-13),
                    Units.inchesToMeters(-13),
                    Units.inchesToMeters(12),
                    new Rotation3d(0.0, -0.4, Units.degreesToRadians(75)));

    public static Transform3d robotToCameraRight =
            new Transform3d(
                    Units.inchesToMeters(-13),
                    Units.inchesToMeters(13),
                    Units.inchesToMeters(12),
                    new Rotation3d(0.0, -0.4, Units.degreesToRadians(-75)));

    public static Transform3d robotToCameraBack =
            new Transform3d(
                    Units.inchesToMeters(-13),
                    Units.inchesToMeters(-13),
                    Units.inchesToMeters(14),
                    new Rotation3d(0.0, -0.4, Units.degreesToRadians(195)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
            new double[] {
                1.0, // Camera 0
                1.0 // Camera 1
            };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available
}
