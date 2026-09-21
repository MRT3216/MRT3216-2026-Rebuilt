package org.mrt3216.objectvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import java.util.Optional;

public class RobotPoseHistory {
    private final TimeInterpolatableBuffer<Pose2d> buffer;

    public RobotPoseHistory(double historySeconds) {
        buffer = TimeInterpolatableBuffer.createBuffer(historySeconds);
    }

    public void record(double timestampSeconds, Pose2d pose) {
        buffer.addSample(timestampSeconds, pose);
    }

    public Optional<Pose2d> sample(double timestampSeconds) {
        return buffer.getSample(timestampSeconds);
    }

    public void clear() {
        buffer.clear();
    }
}
