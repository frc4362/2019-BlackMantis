package com.gemsrobotics.subsystems.pto;

import edu.wpi.first.wpilibj.Solenoid;

public class PTO {
	private final Solenoid m_solenoid;

	public PTO(final PTOConfig ptoConfig) {
		m_solenoid = new Solenoid(ptoConfig.port);
	}

	public void engage() {
		m_solenoid.set(true);
	}

	public void disengage() {
		m_solenoid.set(false);
	}

	public Solenoid getSolenoid() {
		return m_solenoid;
	}
}
