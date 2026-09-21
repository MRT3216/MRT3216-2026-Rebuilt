package org.mrt3216.objectvision;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;

public interface FieldObjectSource {
    List<Translation3d> getPositions();
}
