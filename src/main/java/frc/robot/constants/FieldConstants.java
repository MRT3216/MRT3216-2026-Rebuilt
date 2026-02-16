// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains information for the location of field elements and other useful reference points.
 *
 * <p>All constants are defined relative to the field coordinate system, and from the perspective of
 * the blue alliance station.
 */
public class FieldConstants {
    public static final FieldType fieldType = FieldType.WELDED;

    // --- AprilTag related constants ---
    public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

    // --- Field dimensions ---
    public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
    public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

    /** Vertical field lines found on the field (defined by X-axis offset) */
    public static class LinesVertical {
        public static final double center = fieldLength / 2.0;
        public static final double starting =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX();
        public static final double allianceZone = starting;
        public static final double hubCenter = starting + Hub.width / 2.0;
        public static final double neutralZoneNear = center - Units.inchesToMeters(120);
        public static final double neutralZoneFar = center + Units.inchesToMeters(120);
        public static final double oppHubCenter =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + Hub.width / 2.0;
    }

    /** Horizontal field lines found on the field (defined by Y-axis offset) */
    public static class LinesHorizontal {
        public static final double center = fieldWidth / 2.0;

        // Hub bounds for corner referencing
        public static final double rightBumpStart = Hub.nearRightCorner.getY();
        public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
    }

    /** Hub related constants */
    public static class Hub {
        // Dimensions
        public static final double width = Units.inchesToMeters(47.0);
        public static final double height =
                Units.inchesToMeters(72.0); // includes the catcher at the top
        public static final double innerWidth = Units.inchesToMeters(41.7);
        public static final double innerHeight = Units.inchesToMeters(56.5);

        // --- Units API Wrappers for TurretCalculator ---
        public static final edu.wpi.first.units.measure.Distance FUNNEL_RADIUS =
                Meters.of(innerWidth / 2.0);
        public static final edu.wpi.first.units.measure.Distance FUNNEL_HEIGHT = Meters.of(innerHeight);

        // Relevant reference points on alliance side
        public static final Translation3d topCenterPoint =
                new Translation3d(LinesVertical.hubCenter, fieldWidth / 2.0, height);

        public static final Translation3d innerCenterPoint =
                new Translation3d(LinesVertical.hubCenter, fieldWidth / 2.0, innerHeight);

        public static final Translation2d nearLeftCorner =
                new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d nearRightCorner =
                new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d farLeftCorner =
                new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d farRightCorner =
                new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

        // Relevant reference points on the opposite side
        public static final Translation3d oppTopCenterPoint =
                new Translation3d(LinesVertical.oppHubCenter, fieldWidth / 2.0, height);

        public static final Translation3d oppInnerCenterPoint =
                new Translation3d(LinesVertical.oppHubCenter, fieldWidth / 2.0, innerHeight);

        // Hub faces mapped to AprilTags (Tag 26 is Blue Hub Face)
        public static final Pose2d nearFace =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d();
        public static final Pose2d farFace =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d();
    }

    // --- Loading & Enum Logic ---

    @RequiredArgsConstructor
    public enum FieldType {
        ANDYMARK("andymark"),
        WELDED("welded");

        @Getter private final String jsonFolder;
    }

    public enum AprilTagLayoutType {
        OFFICIAL("2026-official"),
        NONE("2026-none");

        private final String name;
        private volatile AprilTagFieldLayout layout;
        private volatile String layoutString;

        AprilTagLayoutType(String name) {
            this.name = name;
        }

        public AprilTagFieldLayout getLayout() {
            if (layout == null) {
                synchronized (this) {
                    if (layout == null) {
                        try {
                            Path p =
                                    Constants.currentMode == Constants.Mode.SIM
                                            ? Path.of(
                                                    "src",
                                                    "main",
                                                    "deploy",
                                                    "apriltags",
                                                    fieldType.getJsonFolder(),
                                                    name + ".json")
                                            : Path.of(
                                                    Filesystem.getDeployDirectory().getPath(),
                                                    "apriltags",
                                                    fieldType.getJsonFolder(),
                                                    name + ".json");
                            layout = new AprilTagFieldLayout(p);
                            layoutString = new ObjectMapper().writeValueAsString(layout);
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }
                    }
                }
            }
            return layout;
        }

        public String getLayoutString() {
            if (layoutString == null) {
                getLayout();
            }
            return layoutString;
        }
    }
}
