package com.gemsrobotics;

import com.gemsrobotics.subsystems.intake.Manipulator;
import com.gemsrobotics.subsystems.intake.Manipulator.IntakeExtensionState;
import com.gemsrobotics.subsystems.intake.Manipulator.RunMode;
import com.gemsrobotics.util.DualTransmission.Gear;
import com.gemsrobotics.util.joy.Gembutton;
import com.gemsrobotics.util.joy.Gemstick;
import edu.wpi.first.wpilibj.XboxController;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class OperatorInterface {
	private final Gemstick m_stickLeft, m_stickRight;
	private final XboxController m_controller;

	public OperatorInterface(
			final int portLeft,
			final int portRight,
			final int portController
	) {
		m_stickLeft = new Gemstick(portLeft);
		m_stickRight = new Gemstick(portRight);
		m_controller = new XboxController(portController);

		final Gembutton
				shiftUpButton = new Gembutton(m_stickLeft, 1),
				shiftDownButton = new Gembutton(m_stickRight, 1),
				intakeButton = new Gembutton(m_controller, 2),
				exhaustButton = new Gembutton(m_controller, 4),
				deployStage1Button = new Gembutton(m_controller, 6);

		final Runnable intakeNeutralizer = () ->
			  Hardware.getInstance().getManipulator().setSetSpeed(RunMode.NEUTRAL);

		shiftUpButton.whenPressed(() ->
			  Hardware.getInstance().getChassis().getTransmission().set(Gear.HIGH));
		shiftDownButton.whenPressed(() ->
			  Hardware.getInstance().getChassis().getTransmission().set(Gear.LOW));

		intakeButton.whileHeld(() ->
			  Hardware.getInstance().getManipulator().setSetSpeed(RunMode.INTAKING));
		intakeButton.whenReleased(intakeNeutralizer);
		exhaustButton.whileHeld(() ->
			  Hardware.getInstance().getManipulator().setSetSpeed(RunMode.EXHAUSTING));
		exhaustButton.whenReleased(intakeNeutralizer);

		deployStage1Button.whenPressed(() ->
			  Hardware.getInstance().getManipulator().setIntake(IntakeExtensionState.DEPLOYED));
		deployStage1Button.whenReleased(() ->
			  Hardware.getInstance().getManipulator().setIntake(IntakeExtensionState.RETRACTED));
	}

	public Gemstick getStickLeft() {
		return m_stickLeft;
	}

	public Gemstick getStickRight() {
		return m_stickRight;
	}

	public XboxController getController() {
		return m_controller;
	}
}
