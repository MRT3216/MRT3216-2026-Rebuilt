package org.mrt3216.objectvision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;

@FunctionalInterface
public interface TargetSelector {
    Optional<TrackedTarget> select(List<TrackedTarget> tracks, Pose2d robotPose);
}
