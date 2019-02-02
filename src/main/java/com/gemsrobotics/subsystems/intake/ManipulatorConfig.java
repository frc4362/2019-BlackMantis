package com.gemsrobotics.subsystems.intake;

@SuppressWarnings("WeakerAccess")
public class ManipulatorConfig {
	public int stage1Port;
	public int[] stage2Port;

	// DoubleSolenoid Ports
	public int[] extenderPort;
	public int[] placementPort;
	public int[] longPlacePort;
}
