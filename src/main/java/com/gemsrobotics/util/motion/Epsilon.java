package com.gemsrobotics.util.motion;

public enum Epsilon {
	// please note there is nothing here
	;

	public static final double VALUE = 1e-9;

	public static boolean epsilonEquals(final double a, final double b) {
		return (a - VALUE <= b) && (a + VALUE >= b);
	}
}
