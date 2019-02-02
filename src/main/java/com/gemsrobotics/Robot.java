package com.gemsrobotics;

import com.gemsrobotics.commands.DriveCommand;
import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.util.joy.Gemstick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

// TODO make this use BetterTimedRobot
public class Robot extends TimedRobot {
	private OperatorInterface m_oi;

	@Override
	public void robotInit() {
		m_oi = new OperatorInterface(0, 1, 2);

		final var compressor = Hardware.getInstance().getCompressor();
		compressor.setClosedLoopControl(false);
		compressor.stop();
	}

	@Override
	public void teleopInit() {
		final DifferentialDrive chassis = Hardware.getInstance().getChassis();

		Scheduler.getInstance().add(new DriveCommand(chassis, m_oi));

		System.out.format("Speed: %s, %s",
				chassis.getInchesPerSecond(DifferentialDrive.Side.LEFT),
				chassis.getInchesPerSecond(DifferentialDrive.Side.RIGHT));
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}
}
