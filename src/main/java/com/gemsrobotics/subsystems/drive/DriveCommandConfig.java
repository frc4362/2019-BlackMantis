package com.gemsrobotics.subsystems.drive;

public class DriveCommandConfig {
	public double minSpeed, startRampArea, endRampArea,
		slowdownCloseThreshold, slowdownExtendThreshold,
		slowdownOpenThreshold, slowdownResetThreshold,
		slowdownPickupThreshold, stopOverdriveThreshold;

	public double getRampingRange() {
		return 1.0 - minSpeed;
	}

	public double getDivisor() {
		return startRampArea - endRampArea;
	}
}
