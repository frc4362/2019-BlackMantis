package com.gemsrobotics.commands;

import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.util.DualTransmission;
import com.gemsrobotics.util.command.ToggleableCommand;

import static com.gemsrobotics.util.DualTransmission.Gear.LOW;
import static com.gemsrobotics.util.DualTransmission.Gear.HIGH;
import static java.lang.Math.abs;

public class ShiftScheduler extends ToggleableCommand {
	private static final double PEAK_RPM = 5400;

	private final DifferentialDrive m_driveTrain;
	private final DualTransmission m_transmission;

	public ShiftScheduler(final DifferentialDrive driveTrain) {
		m_driveTrain = driveTrain;
		m_transmission = driveTrain.getTransmission();
	}

	@Override
	public void whenEnabled() {
		final var currentGear = m_transmission.get();
		final var shouldShiftHigh = isOverSpeed(.90 * PEAK_RPM);
		final var shouldShiftLow = !isOverSpeed(.80 * PEAK_RPM);

		// TODO add it back
		if (currentGear == LOW && shouldShiftHigh) {
			m_transmission.set(HIGH);
		} else if (currentGear == HIGH && shouldShiftLow) {
			m_transmission.set(LOW);
		}
	}

	private boolean isOverSpeed(final double targetSpeed) {
		final var left = m_driveTrain.getMotor(DifferentialDrive.Side.LEFT).getEncoder().getVelocity();
		final var right = m_driveTrain.getMotor(DifferentialDrive.Side.RIGHT).getEncoder().getVelocity();
		return abs(left) > targetSpeed && abs(right) > targetSpeed;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
