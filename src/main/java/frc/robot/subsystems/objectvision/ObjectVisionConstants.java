package frc.robot.subsystems.objectvision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.mrt3216.objectvision.TargetConfig;

public final class ObjectVisionConstants {
    private ObjectVisionConstants() {}

    // Camera name — must match name configured on coprocessor
    public static final String kCameraName = "Object_Cam";

    // Class ID for FUEL (placeholder — determined by the trained YOLO model)
    public static final int kClassId = 0;

    // FUEL ball: 5.91 in (0.150 m) diameter, center height on carpet
    public static final double kTargetCenterHeightMeters = 0.075;

    // Detection filtering
    public static final double kMinConfidence = 0.5;
    public static final double kMaxRangeMeters = 5.0;
    public static final double kExpirySeconds = 0.5;
    public static final double kAssociationGateMeters = 0.5;

    public static final TargetConfig kFuelConfig =
            new TargetConfig(
                    "FUEL",
                    kClassId,
                    kTargetCenterHeightMeters,
                    kMinConfidence,
                    kMaxRangeMeters,
                    kExpirySeconds,
                    kAssociationGateMeters);

    // Intake faces the FRONT of the robot (0° heading).
    // The driver-assist command (Phase 4) should aim the front at the target.

    // TODO(measure): replace with measured camera transform
    public static final Transform3d robotToCameraObject =
            new Transform3d(0.3, 0.0, 0.5, new Rotation3d(0.0, Math.toRadians(-20.0), 0.0));
}
