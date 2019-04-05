package com.gemsrobotics.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;

@SuppressWarnings("unused")
public class Manipulator {
	private static final CANSparkMaxLowLevel.ConfigParameter SENSOR_PARAMETER =
			CANSparkMaxLowLevel.ConfigParameter.kSensorType;

	private static final int NO_SENSOR_SETTING_ID = 0;

	private final CANSparkMax m_stage2Master, m_stage2Slave;
	private final Solenoid m_arm, m_hand;

	private RunMode m_intakeSetpoint;

	public Manipulator(final ManipulatorConfig config) {
		m_stage2Master = new CANSparkMax(config.stage2Port[0], MotorType.kBrushed);
		m_stage2Master.setIdleMode(CANSparkMax.IdleMode.kBrake);
		disableEncoder(m_stage2Master);

		m_stage2Slave = new CANSparkMax(config.stage2Port[1], MotorType.kBrushed);
		m_stage2Slave.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_stage2Slave.follow(m_stage2Master, true);
		disableEncoder(m_stage2Slave);

		m_arm = new Solenoid(config.placementPort);
		m_hand = new Solenoid(config.longPlacePort);

		m_intakeSetpoint = RunMode.NEUTRAL;
	}

	private void disableEncoder(final CANSparkMax motor) {
		motor.setParameter(SENSOR_PARAMETER, NO_SENSOR_SETTING_ID);
	}

	public enum RunMode {
		INTAKING(-1.0),
		EXHAUSTING(0.45),
		NEUTRAL(-0.25),
		HALTED(0.0);

		private final double speed;

		RunMode(final double s) {
			speed = s;
		}
	}

	public enum PlacementState {
		HOLDING, SHORT_PLACE, LONG_PICKUP, LONG_PLACED
	}

	private void setSpeed(final double speed) {
		m_stage2Master.set(speed);
	}

	public void setSetSpeed(final RunMode mode) {
		m_intakeSetpoint = mode;
		setSpeed(mode.speed);
	}

	public void set(final PlacementState state) {
		switch (state) {
			case SHORT_PLACE:
				m_arm.set(false);
				m_hand.set(true);
				break;

			case LONG_PICKUP:
				m_arm.set(true);
				m_hand.set(true);
				break;

			case LONG_PLACED:
				m_arm.set(false);
				m_hand.set(true);
				break;

			default:
			case HOLDING:
				m_arm.set(false);
				m_hand.set(false);
				break;
		}
	}

	public RunMode getCurrentRunMode() {
		return m_intakeSetpoint;
	}

	public Solenoid getHand() {
		return m_hand;
	}

	public Solenoid getArm() {
		return m_arm;
	}
}
