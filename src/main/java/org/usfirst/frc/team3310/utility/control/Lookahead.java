package org.usfirst.frc.team3310.utility.control;

import static java.lang.Math.max;
import static java.lang.Math.min;

/**
 * A utility class for interpolating lookahead distance based on current speed.
 */
public class Lookahead {
    public final double min_distance;
    public final double max_distance;
    public final double min_speed;
    public final double max_speed;

    protected final double delta_distance;
    protected final double delta_speed;

    public Lookahead(
            final double min_distance,
            final double max_distance,
            final double min_speed,
            final double max_speed
    ) {
        this.min_distance = min_distance;
        this.max_distance = max_distance;
        this.min_speed = min_speed;
        this.max_speed = max_speed;
        delta_distance = max_distance - min_distance;
        delta_speed = max_speed - min_speed;
    }

    public double getLookaheadForSpeed(final double speed) {
        final double lookaheadDistance = delta_distance * (speed - min_speed) / delta_speed + min_distance;
        return Double.isNaN(lookaheadDistance) ? min_distance : max(min_distance, min(max_distance, lookaheadDistance));
    }
}
