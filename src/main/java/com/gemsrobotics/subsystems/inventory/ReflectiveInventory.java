package com.gemsrobotics.subsystems.inventory;

import edu.wpi.first.wpilibj.AnalogInput;

public class ReflectiveInventory extends Inventory {
	private static final double DETECTION_THRESHOLD = 0.2;

	private final AnalogInput m_reflectiveSensor;

	public ReflectiveInventory(final int port) {
		m_reflectiveSensor = new AnalogInput(port);
	}

	@Override
	public boolean hasPanel() {
		return !hasCargo();
	}

	@Override
	public boolean hasCargo() {
		return m_reflectiveSensor.getVoltage() > DETECTION_THRESHOLD;
	}
}
