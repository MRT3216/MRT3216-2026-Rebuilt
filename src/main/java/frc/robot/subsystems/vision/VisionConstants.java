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

public final class VisionConstants {
    private VisionConstants() {}

    // AprilTag layout — final so the JVM initializes it exactly once at class-load time
    public static final AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static final String cameraLeftName = "Left_Cam";
    public static final String cameraRightName = "Right_Cam";
    public static final String cameraFrontName = "Front_Cam";
    public static final String cameraBackName = "Rear_Cam";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    // NOTE: Rotation3d args mix Degrees and Radians — this is intentional (pitch is in radians).
    public static final Transform3d robotToCameraLeft =
            new Transform3d(
                    Inches.of(-11.791238),
                    Inches.of(12.876205),
                    Inches.of(14.154937),
                    new Rotation3d(Degrees.of(0.0), Radians.of(0.4), Degrees.of(75)));

    public static final Transform3d robotToCameraRight =
            new Transform3d(
                    Inches.of(-11.625209),
                    Inches.of(-12.340245),
                    Inches.of(14.169104),
                    new Rotation3d(Degrees.of(0), Radians.of(0.4), Degrees.of(285)));

    public static final Transform3d robotToCameraFront =
            new Transform3d(
                    Inches.of(-11.234075),
                    Inches.of(-10.772856),
                    Inches.of(20.375546),
                    new Rotation3d(Degrees.of(0), Radians.of(0.4), Degrees.of(15)));

    public static final Transform3d robotToCameraBack =
            new Transform3d(
                    Inches.of(-12.865422),
                    Inches.of(11.805673),
                    Inches.of(16.151008),
                    new Rotation3d(Degrees.of(0), Radians.of(0.4), Degrees.of(195)));

    // Basic filtering thresholds
    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double linearStdDevBaseline = 0.02; // Meters
    public static final double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] cameraStdDevFactors =
            new double[] {
                1.0, // Camera 0
                1.0 // Camera 1
            };

    // Multipliers to apply for MegaTag 2 observations
    public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static final double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available
}
