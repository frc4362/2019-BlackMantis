package com.gemsrobotics.subsystems.drive;

@SuppressWarnings("WeakerAccess")
public class DrivePorts {
	int front_left, back_left, front_right, back_right;

	public Integer[] get() {
		return new Integer[] { front_left, back_left, front_right, back_right };
	}
}
