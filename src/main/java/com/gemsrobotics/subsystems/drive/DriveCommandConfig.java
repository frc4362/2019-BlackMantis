package com.gemsrobotics.subsystems.drive;

import com.gemsrobotics.util.DualTransmission;

public class DriveCommandConfig {
	public double minSpeed, startRampArea, endRampArea,
		slowdownCloseThreshold, slowdownExtendThreshold,
		slowdownOpenThreshold, slowdownResetThreshold,
		slowdownPickupThreshold, stopOverdriveThreshold,
		rumbleAngularPowerRequirement;

	public SlowingSchema lowGearSlowing, highGearSlowing;

	public SlowingSchema schemaForGear(final DualTransmission.Gear gear) {
		if (gear == DualTransmission.Gear.HIGH) {
			return highGearSlowing;
		} else {
			return lowGearSlowing;
		}
	}
}
