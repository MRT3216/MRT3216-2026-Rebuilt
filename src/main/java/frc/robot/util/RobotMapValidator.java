package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Simple validator that scans {@code frc.robot.constants.RobotMap} for duplicated integer IDs and
 * reports warnings to DriverStation. Intended to run once at robot startup.
 */
public final class RobotMapValidator {
    private RobotMapValidator() {}

    /**
     * Scans nested classes under {@code frc.robot.constants.RobotMap} for public static integer
     * fields and reports duplicates.
     */
    public static void validate() {
        try {
            Class<?> root = Class.forName("frc.robot.constants.RobotMap");
            Map<Integer, List<String>> ids = new HashMap<>();
            collectIdsRecursive(root, ids, root.getSimpleName());

            StringBuilder dupMsg = new StringBuilder();
            for (Map.Entry<Integer, List<String>> e : ids.entrySet()) {
                List<String> names = e.getValue();
                if (names.size() > 1) {
                    dupMsg.append(String.format("ID %d used by: %s\n", e.getKey(), String.join(", ", names)));
                }
            }

            if (dupMsg.length() > 0) {
                String msg = "RobotMap ID conflicts detected:\n" + dupMsg.toString();
                DriverStation.reportWarning(msg, false);
            } else {
                // Report informational success via DriverStation (non-fatal)
                DriverStation.reportWarning("RobotMapValidator: no duplicate IDs found.", false);
            }
        } catch (ClassNotFoundException ex) {
            DriverStation.reportWarning("RobotMapValidator: RobotMap class not found", false);
        } catch (Exception ex) {
            DriverStation.reportWarning(
                    "RobotMapValidator: unexpected error - " + ex.getMessage(), false);
        }
    }

    private static void collectIdsRecursive(
            Class<?> cls, Map<Integer, List<String>> ids, String prefix) {
        // Collect integer fields in this class
        for (Field f : cls.getDeclaredFields()) {
            int mods = f.getModifiers();
            if (Modifier.isStatic(mods) && Modifier.isPublic(mods)) {
                Class<?> t = f.getType();
                if (t == int.class || t == Integer.class) {
                    try {
                        Object val = f.get(null);
                        if (val != null) {
                            int iv = ((Number) val).intValue();
                            String name = prefix + "." + f.getName();
                            ids.computeIfAbsent(iv, k -> new ArrayList<>()).add(name);
                        }
                    } catch (IllegalAccessException ignored) {
                    }
                }
            }
        }

        // Recurse into nested classes
        for (Class<?> nested : cls.getDeclaredClasses()) {
            collectIdsRecursive(nested, ids, prefix + "." + nested.getSimpleName());
        }
    }
}
