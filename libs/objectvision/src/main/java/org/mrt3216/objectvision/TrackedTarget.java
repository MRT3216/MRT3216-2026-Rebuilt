package org.mrt3216.objectvision;

import edu.wpi.first.math.geometry.Translation2d;

public record TrackedTarget(
        Translation2d position, double confidence, double lastUpdateTimestamp, int trackId) {}
