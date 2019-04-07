package com.gemsrobotics.util;

import edu.wpi.first.wpilibj.Solenoid;

public class DualTransmission {
	public enum Gear {
		HIGH(6.6287, 0), LOW(16.3636, 0);

		public final double ratio, maxRPM;

		Gear(final double r, final double rpm) {
			ratio = r;
			maxRPM = rpm;
		}
	}

	private final Solenoid m_shifter;

	public DualTransmission(final Solenoid shifter) {
		m_shifter = shifter;
	}

	public Gear get() {
		return m_shifter.get() ? Gear.LOW : Gear.HIGH;
	}

	public void set(final Gear gear) {
		m_shifter.set(gear == Gear.HIGH);
	}
}
