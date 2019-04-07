package com.gemsrobotics.commands;

import com.gemsrobotics.util.camera.Limelight;
import edu.wpi.first.wpilibj.command.Command;

@SuppressWarnings("WeakerAccess")
public class WaitForAreaCommand extends Command {
	private final double m_threshold;
	private final Limelight m_limelight;

	public WaitForAreaCommand(final Limelight limelight, final double threshold) {
		m_limelight = limelight;
		m_threshold = threshold;
	}

	@Override
	public void execute() { }

	@Override
	public boolean isFinished() {
		return m_limelight.getArea() > m_threshold;
	}
}
