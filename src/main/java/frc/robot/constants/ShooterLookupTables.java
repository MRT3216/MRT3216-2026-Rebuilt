package frc.robot.constants;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * Loader for shooter lookup tables. Tries to load `shooter_lookup.json` from resources. If not
 * present, falls back to embedded defaults.
 */
public final class ShooterLookupTables {
    private ShooterLookupTables() {}

    public static final double[][] HUB;
    public static final double[][] PASS;

    static {
        double[][] hubDefault = {
            {1.0, 80.0, 75.0, 0.45},
            {2.0, 82.5, 72.0, 0.65},
            {3.0, 85.0, 68.0, 0.85},
            {4.0, 90.0, 65.0, 1.05},
            {5.0, 95.0, 62.0, 1.25},
            {6.0, 105.0, 60.0, 1.45}
        };
        double[][] passDefault = {{1.0, 75.0, 54.0, 0.35}, {5.5, 78.3, 45.0, 1.25}};

        double[][] hub = hubDefault;
        double[][] pass = passDefault;

        try {
            InputStream is = ShooterLookupTables.class.getResourceAsStream("/shooter_lookup.json");
            if (is != null) {
                StringBuilder sb = new StringBuilder();
                try (BufferedReader r =
                        new BufferedReader(new InputStreamReader(is, StandardCharsets.UTF_8))) {
                    String line;
                    while ((line = r.readLine()) != null) {
                        sb.append(line);
                    }
                }
                String json = sb.toString();
                hub = parseArray(json, "HUB", hubDefault);
                pass = parseArray(json, "PASS", passDefault);
            }
        } catch (Exception e) {
            // on any parse/load error, fall back to defaults
            hub = hubDefault;
            pass = passDefault;
        }

        HUB = hub;
        PASS = pass;
    }

    // Very small JSON extractor that finds the named array of arrays and parses numbers.
    private static double[][] parseArray(String json, String key, double[][] fallback) {
        try {
            int idx = json.indexOf('"' + key + '"');
            if (idx < 0) return fallback;
            int start = json.indexOf('[', idx);
            if (start < 0) return fallback;
            int end = findMatchingBracket(json, start);
            if (end < 0) return fallback;
            String arrayText = json.substring(start, end + 1);

            // remove whitespace and outer brackets
            arrayText = arrayText.trim();
            if (arrayText.startsWith("[")) arrayText = arrayText.substring(1);
            if (arrayText.endsWith("]")) arrayText = arrayText.substring(0, arrayText.length() - 1);

            List<double[]> rows = new ArrayList<>();
            int i = 0;
            while (i < arrayText.length()) {
                int open = arrayText.indexOf('[', i);
                if (open < 0) break;
                int close = arrayText.indexOf(']', open);
                if (close < 0) break;
                String row = arrayText.substring(open + 1, close).trim();
                if (!row.isEmpty()) {
                    String[] parts = row.split(",");
                    double[] vals = new double[parts.length];
                    for (int j = 0; j < parts.length; j++) vals[j] = Double.parseDouble(parts[j].trim());
                    rows.add(vals);
                }
                i = close + 1;
            }
            double[][] out = new double[rows.size()][];
            for (int r = 0; r < rows.size(); r++) out[r] = rows.get(r);
            return out.length > 0 ? out : fallback;
        } catch (Exception e) {
            return fallback;
        }
    }

    private static int findMatchingBracket(String s, int openIdx) {
        int depth = 0;
        for (int i = openIdx; i < s.length(); i++) {
            char c = s.charAt(i);
            if (c == '[') depth++;
            else if (c == ']') {
                depth--;
                if (depth == 0) return i;
            }
        }
        return -1;
    }
}
