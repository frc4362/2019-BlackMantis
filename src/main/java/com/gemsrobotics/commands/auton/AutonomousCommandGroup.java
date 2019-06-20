package com.gemsrobotics.commands.auton;

import com.gemsrobotics.OperatorInterface;
import edu.wpi.first.wpilibj.command.CommandGroup;

import static java.lang.Math.abs;

public class AutonomousCommandGroup extends CommandGroup {
	private OperatorInterface m_oi;

	public AutonomousCommandGroup(final OperatorInterface oi) {
		m_oi = oi;
	}

	@Override
	public boolean isFinished() {
		return super.isFinished() || abs(m_oi.getStickRight().getMagnitude()) > 0.2;
	}
}
