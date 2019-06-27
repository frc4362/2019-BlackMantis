package com.gemsrobotics.util.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LazyTalonSRX extends TalonSRX {
	private ControlMode m_lastMode;
	private double m_lastValue;

	public LazyTalonSRX(final int port) {
		super(port);

		m_lastMode = null;
		m_lastValue = Double.NaN;
	}

	@Override
	public void set(final ControlMode mode, final double value) {
		if (mode != m_lastMode || value != m_lastValue) {
			super.set(mode, value);

			m_lastMode = mode;
			m_lastValue = value;
		}
	}
}
