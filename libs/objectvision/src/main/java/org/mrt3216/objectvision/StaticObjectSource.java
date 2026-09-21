package org.mrt3216.objectvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class StaticObjectSource implements FieldObjectSource {
    private final ArrayList<Translation3d> positions = new ArrayList<>();

    @Override
    public List<Translation3d> getPositions() {
        return Collections.unmodifiableList(positions);
    }

    public void add(Translation3d position) {
        positions.add(position);
    }

    public void remove(Translation3d position) {
        positions.remove(position);
    }

    /** Remove all objects within a rectangle centered on the robot pose. */
    public void removeWithin(Pose2d robot, double halfLength, double halfWidth) {
        double cos = robot.getRotation().getCos();
        double sin = robot.getRotation().getSin();
        double rx = robot.getX();
        double ry = robot.getY();

        for (int i = positions.size() - 1; i >= 0; i--) {
            Translation3d pos = positions.get(i);
            // Transform to robot-relative coordinates
            double dx = pos.getX() - rx;
            double dy = pos.getY() - ry;
            double localX = cos * dx + sin * dy;
            double localY = -sin * dx + cos * dy;

            if (Math.abs(localX) < halfLength && Math.abs(localY) < halfWidth) {
                positions.remove(i);
            }
        }
    }

    public void clear() {
        positions.clear();
    }

    public int size() {
        return positions.size();
    }
}
