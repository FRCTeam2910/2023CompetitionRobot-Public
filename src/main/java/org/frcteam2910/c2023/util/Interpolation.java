package org.frcteam2910.c2023.util;

/**
 * Contains interpolation methods.
 */
public class Interpolation {
    /**
     * Gives me the "t" value needed to calculate moment of inertia
     * @param startValue Minimum extension length
     * @param endValue Maximum extension length
     * @param x Current extension length
     * @return The t value
     */
    public static double inverseInterpolate(double startValue, double endValue, double x) {
        return (x - startValue) / (endValue - startValue);
    }
}
