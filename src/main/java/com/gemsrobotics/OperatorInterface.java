package com.gemsrobotics;

import com.gemsrobotics.subsystems.lift.Lift;
import com.gemsrobotics.subsystems.manipulator.Manipulator.IntakeExtensionState;
import com.gemsrobotics.subsystems.manipulator.Manipulator.RunMode;
import com.gemsrobotics.util.DualTransmission.Gear;
import com.gemsrobotics.util.joy.Gembutton;
import com.gemsrobotics.util.joy.Gemstick;
import com.gemsrobotics.util.joy.Gemstick.POVState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.POVButton;

import static com.gemsrobotics.util.command.Commands.commandOf;

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
				shiftDownButton = new Gembutton(m_stickLeft, 1),
				shiftUpButton = new Gembutton(m_stickRight, 4),
				intakeButton = new Gembutton(m_controller, 1),
				exhaustButton = new Gembutton(m_controller, 4),
				deployStage1Button = new Gembutton(m_controller, 3),
				punchHoldButton = new Gembutton(m_controller, 5),
				handToggler = new Gembutton(m_controller, 6);

		final POVButton
				cargoShipHeightButton = new POVButton(m_controller, POVState.W.toDegrees()),
				height1Button = new POVButton(m_controller, POVState.S.toDegrees()),
				height2Button = new POVButton(m_controller, POVState.E.toDegrees()),
				height3Button = new POVButton(m_controller, POVState.N.toDegrees());

		final Runnable intakeNeutralizer = () ->
			  Hardware.getInstance().getManipulator().setSetSpeed(RunMode.NEUTRAL);

		final var transmission = Hardware.getInstance().getChassis().getTransmission();
		final var manipulator = Hardware.getInstance().getManipulator();
		final var inventory = Hardware.getInstance().getInventory();
		final var lift = Hardware.getInstance().getLift();

		shiftUpButton.whenPressed(() ->
			  transmission.set(Gear.HIGH));
		shiftDownButton.whenPressed(() ->
			  transmission.set(Gear.LOW));

		intakeButton.whileHeld(() ->
			  manipulator.setSetSpeed(RunMode.INTAKING));
		intakeButton.whenReleased(intakeNeutralizer);

		exhaustButton.whileHeld(() ->
			  manipulator.setSetSpeed(RunMode.EXHAUSTING));
		exhaustButton.whenReleased(intakeNeutralizer);

		deployStage1Button.setToggle(
				() -> manipulator.setIntake(IntakeExtensionState.DEPLOYED),
				() -> manipulator.setIntake(IntakeExtensionState.RETRACTED));

		punchHoldButton.whenPressed(() ->
			manipulator.getLongPlacer().set(true));
		punchHoldButton.whenReleased(() ->
			manipulator.getLongPlacer().set(false));
		handToggler.whenPressed(
				() -> manipulator.getPlacer().set(true));
		handToggler.whenReleased(
				() -> manipulator.getPlacer().set(false));

		cargoShipHeightButton.whenPressed(commandOf(() -> {
			if (inventory.hasCargo()) {
				System.out.println("Cargo detected!");
				lift.setPreset(Lift.Position.CARGO_SHIP);
			} else {
				System.out.println("Cargo ship height attempted; but no cargo detected!");
			}
		}));

		height1Button.whenPressed(commandOf(() -> {
			switch (inventory.getCurrentPiece()) {
				case PANEL:
					lift.setPreset(Lift.Position.PANEL_1);
					System.out.println("Moved to PANEL_1");
					break;
				case CARGO:
					lift.setPreset(Lift.Position.CARGO_1);
					System.out.println("Moved to CARGO_1");
					break;
				case NONE:
				default:
					System.out.println("Setpoint commanded, but no gamepiece found!");
					break;
			}
		}));

		height2Button.whenPressed(commandOf(() -> {
			switch (inventory.getCurrentPiece()) {
				case PANEL:
					lift.setPreset(Lift.Position.PANEL_2);
					System.out.println("Moved to PANEL_2");
					break;
				case CARGO:
					lift.setPreset(Lift.Position.CARGO_2);
					System.out.println("Moved to CARGO_2");
					break;
				case NONE:
				default:
					System.out.println("Setpoint commanded, but no gamepiece found!");
					break;
			}
		}));

		height3Button.whenPressed(commandOf(() -> {
			switch (inventory.getCurrentPiece()) {
				case PANEL:
					lift.setPreset(Lift.Position.PANEL_3);
					System.out.println("Moved to PANEL_3");
					break;
				case CARGO:
					lift.setPreset(Lift.Position.CARGO_3);
					System.out.println("Moved to CARGO_3");
					break;
				case NONE:
				default:
					System.out.println("Setpoint commanded, but no gamepiece found!");
					break;
			}
		}));
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
