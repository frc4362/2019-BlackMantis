package com.gemsrobotics.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Manipulator {
	private final CANSparkMax m_stage1, m_stage2Master, m_stage2Slave;
	private final DoubleSolenoid m_intakeExtender, m_placer, m_longPlacer;

	public Manipulator(final ManipulatorConfig config) {
		// TODO figure out how to use brushed CanSparkMax
		m_stage1 = new CANSparkMax(config.stage1Port, MotorType.kBrushed);

		m_stage2Master = new CANSparkMax(config.stage2Port[0], MotorType.kBrushed);
		m_stage2Master.setIdleMode(CANSparkMax.IdleMode.kBrake);

		m_stage2Slave = new CANSparkMax(config.stage2Port[1], MotorType.kBrushed);
		m_stage2Slave.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_stage2Slave.follow(m_stage2Master, true);

		m_intakeExtender = new DoubleSolenoid(config.extenderPort[0], config.extenderPort[1]);
		m_placer = new DoubleSolenoid(config.placementPort[0], config.placementPort[1]);
		m_longPlacer = new DoubleSolenoid(config.longPlacePort[0], config.longPlacePort[1]);
	}

	public enum IntakeExtensionState {
		DEPLOYED(Value.kForward),
		RETRACTED(Value.kReverse);

		public final Value value;

		IntakeExtensionState(final Value val) {
			value = val;
		}
	}

	public enum RunMode {
		INTAKING(-1.0),
		EXHAUSTING(1.0),
		NEUTRAL(0.0);

		private final double speed;

		RunMode(final double s) {
			speed = s;
		}
	}

	public enum PlacementState {
		HOLDING, SHORT_PLACE, LONG_PLACE
	}

	private void setSpeed(final double speed) {
		m_stage1.set(speed);
		m_stage2Master.set(speed);
	}

	public void setSetSpeed(final RunMode mode) {
		setSpeed(mode.speed);
	}

	public void setIntake(final IntakeExtensionState state) {
		m_intakeExtender.set(state.value);
	}

	public void setPlacer(final PlacementState state) {
		switch (state) {
			case SHORT_PLACE:
				m_placer.set(Value.kForward);
				m_longPlacer.set(Value.kForward);
				break;

			case LONG_PLACE:
				m_placer.set(Value.kForward);
				m_longPlacer.set(Value.kForward);
				break;

			default:
			case HOLDING:
				m_placer.set(Value.kReverse);
				m_longPlacer.set(Value.kReverse);
				break;
		}
	}
}
