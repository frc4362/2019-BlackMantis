package com.gemsrobotics.commands;

import com.gemsrobotics.util.camera.Limelight;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

@SuppressWarnings("WeakerAccess")
public class WaitForLimelightArea extends Command {
	private final double m_threshold;
	private final Limelight m_limelight;
	private final XboxController m_controller;

	private int m_runs;

	public WaitForLimelightArea(
			final Limelight limelight,
			final XboxController controller,
			final double threshold
	) {
		m_limelight = limelight;
		m_controller = controller;
		m_threshold = threshold;
	}

	@Override
	public void initialize() {
		m_runs = 0;
	}

	@Override
	public void execute() {
		if (m_controller.getPOV() == -1 && m_runs > 4) {
			getGroup().cancel(); // TODO it might be this
		}

		m_runs++;
	}

	@Override
	public boolean isFinished() {
		return m_limelight.getArea() > m_threshold;
	}
}
