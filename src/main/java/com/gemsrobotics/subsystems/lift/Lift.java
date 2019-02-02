package com.gemsrobotics.subsystems.lift;

public abstract class Lift {
	public enum Position {
		TOP(1.0),
		MIDDLE(0.5),
		BOTTOM(0.0);

		public final double percent;

		/**
		 * @param p The percent extension to put the lift at
		 */
		Position(final double p) {
			percent = p;
		}
	}

	public abstract void setPreset(Position position);
	public abstract void adjustPosition(double percent);
}
