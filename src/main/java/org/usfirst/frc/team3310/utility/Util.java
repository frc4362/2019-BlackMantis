package org.usfirst.frc.team3310.utility;

import java.util.Collection;
import java.util.List;

import static java.lang.Math.max;
import static java.lang.Math.min;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    /** Prevent this class from being instantiated. */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double top, double bot) {
        return min(top, max(bot, v));
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        final StringBuilder sb = new StringBuilder();

        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }

        return sb.toString();
    }

    public static boolean epsilonEquals(final double a, final double b, final double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final Collection<Double> list, final double value, final double epsilon) {
        return list.stream().allMatch(n -> epsilonEquals(n, value, epsilon));
    }
}
