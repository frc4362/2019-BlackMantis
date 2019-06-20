package com.gemsrobotics.util;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;

public class PIDF {
	public double kP, kI, kD, kF, kIZ;

	public void configure(final CANPIDController controller, final int port) {
		controller.setP(kP, port);
		controller.setI(kI, port);
		controller.setD(kD, port);
		controller.setFF(kF, port);
		controller.setIZone(kIZ, port);
	}

	public void configure(final CANPIDController controller) {
		configure(controller, 0);
	}
}
