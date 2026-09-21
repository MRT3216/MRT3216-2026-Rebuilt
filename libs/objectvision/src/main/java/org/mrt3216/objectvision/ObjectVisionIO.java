package org.mrt3216.objectvision;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectVisionIO {
    @AutoLog
    public static class ObjectVisionIOInputs {
        public boolean connected = false;
        public DetectionObservation[] detections = new DetectionObservation[0];
    }

    public static record DetectionObservation(
            int classId,
            double confidence,
            double yawRadians,
            double pitchRadians,
            double areaPercent,
            double timestampSeconds) {}

    public default void updateInputs(ObjectVisionIOInputs inputs) {}
}
