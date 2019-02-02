package com.gemsrobotics.commands;

import com.gemsrobotics.OperatorInterface;
import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.util.joy.Gemstick;
import edu.wpi.first.wpilibj.command.Command;

public class DriveCommand extends Command {
	private final DifferentialDrive m_chassis;
	private final Gemstick m_stickLeft, m_stickRight;
	private DriveMode m_mode;

	public enum DriveMode {
		CURVATURE, TANK, VISION, NONE
	}

	public DriveCommand(
			final DifferentialDrive chassis,
			final OperatorInterface oi
	) {
		m_chassis = chassis;
		m_mode = DriveMode.CURVATURE;
		m_stickLeft = oi.getStickLeft();
		m_stickRight = oi.getStickRight();
	}

	public void setDriveMode(final DriveMode mode) {
		m_mode = mode;
	}

	@Override
	public void execute() {
		switch (m_mode) {
			case CURVATURE:
				final double linearPower = m_stickLeft.get(Gemstick.Lens.Y);
				final double angularPower = m_stickRight.get(Gemstick.Lens.X);
				final boolean rawButton = m_stickRight.getRawButton(2);
				m_chassis.curvatureDrive(linearPower, angularPower, rawButton);
				break;

			case TANK:
				final double leftPower = m_stickLeft.get(Gemstick.Lens.Y);
				final double rightPower = m_stickRight.get(Gemstick.Lens.Y);
				m_chassis.drive(leftPower, rightPower);
				break;

			case VISION:
				throw new RuntimeException("Not ready yet!");

			default:
			case NONE:
				m_chassis.stopMotors();
				break;
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
