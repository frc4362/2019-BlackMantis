package com.gemsrobotics.util.math;

public interface Interpolate<T> {
	T interpolate(final T other, final double n);
}
