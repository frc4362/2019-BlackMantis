package com.gemsrobotics.subsystems.adjuster;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.gemsrobotics.util.PIDF;
import com.gemsrobotics.util.motion.Rotation;
import com.gemsrobotics.util.motion.Translation;

import static java.lang.Math.max;
import static java.lang.Math.min;

@SuppressWarnings("unused")
public class LateralAdjuster {
	public final double kLatVolts;
	private final WPI_TalonSRX m_motor;
	private final double m_widthTicks, m_widthInches;

	public LateralAdjuster(final LateralAdjusterConfig cfg) {
		kLatVolts = cfg.nominalVolts;

		m_motor = new WPI_TalonSRX(cfg.port);

		final PIDF pids = cfg.pidVars;
		m_motor.config_kP(0, pids.kP,0);
		m_motor.config_kI(0, pids.kI,0);
		m_motor.config_kD(0, pids.kD,0);
		m_motor.config_kF(0, pids.kF,0);

		m_motor.setNeutralMode(NeutralMode.Brake);

		m_motor.configForwardSoftLimitEnable(true, 0);
		m_motor.configReverseSoftLimitEnable(true, 0);
		m_motor.configForwardSoftLimitThreshold(cfg.leftDist, 0);
		m_motor.configReverseSoftLimitThreshold(cfg.rightDist, 0);

		m_widthTicks = cfg.leftDist - cfg.rightDist;
		m_widthInches = cfg.widthInches;
	}

	private static double constrain(
			final double bot,
			final double val,
			final double top
	) {
		return max(bot, min(val, top));
	}

	private void setTicks(double ticks) {
		m_motor.set(ControlMode.Position, constrain(-m_widthTicks / 2, ticks, m_widthTicks / 2));
	}

	public void set(final double percent) {
		setTicks(percent * m_widthTicks - m_widthTicks / 2);
	}

	public void drive(final double speed) {
		m_motor.set(speed);
	}

	public void stop() {
		m_motor.set(ControlMode.PercentOutput, 0);
	}

	public void align(final Translation trans) {
		final double proportion = (trans.y() / m_widthInches) * m_widthTicks;
		final double adjusted = -(proportion - (m_widthTicks / 2));
		setTicks(adjusted);
	}

	public WPI_TalonSRX getMotor() {
		return m_motor;
	}

	public double getPosition() {
		return m_motor.getSelectedSensorPosition(0);
	}
}
