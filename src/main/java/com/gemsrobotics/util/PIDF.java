package com.gemsrobotics.util;

import com.revrobotics.CANPIDController;

public class PIDF {
	public double kP, kI, kD, kF, kIZ;

	public void configure(final CANPIDController controller) {
		controller.setP(kP);
		controller.setI(kI);
		controller.setD(kD);
		controller.setFF(kF);
		controller.setIZone(kIZ);
	}
}
