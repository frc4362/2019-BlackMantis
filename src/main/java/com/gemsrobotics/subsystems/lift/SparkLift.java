package com.gemsrobotics.subsystems.lift;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import java.util.Arrays;

public class SparkLift extends Lift {
	private final CANSparkMax m_motorMaster, m_motorSlave;
	private final LiftConfig m_liftConfig;
	private double m_setpoint;

	public SparkLift(final LiftConfig liftConfig) {
		m_liftConfig = liftConfig;

		m_motorMaster = new CANSparkMax(m_liftConfig.portMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
		m_motorSlave = new CANSparkMax(m_liftConfig.portSlave, CANSparkMaxLowLevel.MotorType.kBrushless);

		Arrays.asList(m_motorMaster, m_motorSlave).forEach(motor ->
		   m_liftConfig.pidVars.configure(motor.getPIDController()));

		m_motorMaster.setInverted(false);
		m_motorSlave.setInverted(true);
		m_motorSlave.follow(m_motorMaster);

		m_setpoint = 0.0;
	}

	private double toRotations(final double percent) {
		return percent * m_liftConfig.totalRotations() + m_liftConfig.rotationsBottom;
	}

	private void setRotations(final double rotations) {
		m_setpoint = rotations;
		m_motorMaster.getPIDController().setReference(m_setpoint, ControlType.kPosition);
	}

	@Override
	public void setPreset(final Position pos) {
		setRotations(toRotations(pos.percent));
	}

	@Override
	public void adjustPosition(final double percent) {
		setRotations(m_setpoint + toRotations(percent));
	}
}
