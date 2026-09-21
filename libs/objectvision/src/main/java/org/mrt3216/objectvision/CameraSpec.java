package org.mrt3216.objectvision;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraSpec(ObjectVisionIO io, Transform3d robotToCamera) {}
