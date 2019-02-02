package com.gemsrobotics.util;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DualTransmission {
	public enum Gear {
		HIGH(6.6287, 0), LOW(16.3636, 0);

		public final double ratio, maxRPM;

		Gear(final double r, final double rpm) {
			ratio = r;
			maxRPM = rpm;
		}
	}

	private final DoubleSolenoid m_shifter;

	public DualTransmission(final DoubleSolenoid shifter) {
		m_shifter = shifter;
	}

	public Gear get() {
		return m_shifter.get() == Value.kForward ? Gear.HIGH : Gear.LOW;
	}

	public void set(final Gear gear) {
		m_shifter.set(gear == Gear.HIGH ? Value.kForward : Value.kReverse);
	}
}
