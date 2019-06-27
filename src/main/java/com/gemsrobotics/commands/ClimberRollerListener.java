package com.gemsrobotics.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

public class ClimberRollerListener extends Command {
	private TalonSRX m_rollers;
	private XboxController m_controller;

	private boolean m_hasRun;

	public ClimberRollerListener(
			final TalonSRX rollers,
			final XboxController controller
	) {
		m_hasRun = false;

		m_rollers = rollers;
		m_controller = controller;
	}

	private static double deadband(final double val) {
		return Math.abs(val) > 0.2 ? val : 0.0;
	}

	@Override
	public void initialize() {
		m_hasRun = true;
	}

	@Override
	public void execute() {
		m_rollers.set(ControlMode.PercentOutput, deadband(m_controller.getY(Hand.kLeft)));
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public void reset() {
		m_hasRun = false;
	}

	public boolean hasPreviouslyRun() {
		return m_hasRun;
	}
}
