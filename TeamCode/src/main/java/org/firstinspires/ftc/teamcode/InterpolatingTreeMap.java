package org.firstinspires.ftc.teamcode;

import java.util.TreeMap;

/**
 * A standalone interpolating lookup table using Java's TreeMap.
 * Automatically performs linear interpolation between defined calibration points.
 */
public class InterpolatingTreeMap extends TreeMap<Double, Double> {

    /**
     * Retrieves the interpolated value for a given key.
     *
     * @param key The input variable (e.g., distance in inches)
     * @return The interpolated output value (e.g., target velocity)
     */
    public double getInterpolated(double key) {
        if (isEmpty()) return 0.0;

        // If exact key exists, return it immediately
        if (containsKey(key)) {
            return get(key);
        }

        Double floorKey = floorKey(key);
        Double ceilingKey = ceilingKey(key);

        // If lower than all points, return the lowest point (clamping)
        if (floorKey == null) {
            return get(ceilingKey);
        }
        // If higher than all points, return the highest point (clamping)
        if (ceilingKey == null) {
            return get(floorKey);
        }

        // Linear interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        double x1 = floorKey;
        double y1 = get(floorKey);
        double x2 = ceilingKey;
        double y2 = get(ceilingKey);

        return y1 + (key - x1) * (y2 - y1) / (x2 - x1);
    }
}