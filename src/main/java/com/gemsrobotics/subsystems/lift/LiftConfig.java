package com.gemsrobotics.subsystems.lift;

import com.gemsrobotics.util.PIDF;

@SuppressWarnings("WeakerAccess")
public class LiftConfig {
	int portMaster, portSlave;
	double rotationsTop, rotationsBottom;
	double cameraBlockedTop, cameraBlockedBottom;
	PIDF pidVars;

	public double totalRotations() {
		return rotationsTop - rotationsBottom;
	}
}
