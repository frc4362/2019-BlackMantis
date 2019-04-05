package com.gemsrobotics.subsystems.adjuster;

import com.gemsrobotics.util.PIDF;

@SuppressWarnings({"unused", "WeakerAccess"})
public class LateralAdjusterConfig {
	public int port, leftDist, rightDist;
	public double widthInches, nominalVolts, adjustmentThresholdRadians;
	public PIDF pidVars;
}
