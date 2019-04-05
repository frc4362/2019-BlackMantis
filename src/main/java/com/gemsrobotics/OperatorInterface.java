package com.gemsrobotics;

import com.gemsrobotics.commands.AutoPickupCommand;
import com.gemsrobotics.commands.AutoPlaceFactory;
import com.gemsrobotics.commands.ClimberRollerListener;
import com.gemsrobotics.subsystems.lift.Lift;
import com.gemsrobotics.subsystems.manipulator.Manipulator.RunMode;
import com.gemsrobotics.util.DualTransmission.Gear;
import com.gemsrobotics.util.joy.Gembutton;
import com.gemsrobotics.util.joy.Gemstick;
import com.gemsrobotics.util.joy.Gemstick.POVState;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Scheduler;

import java.util.Objects;

import static com.gemsrobotics.util.command.Commands.commandOf;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class OperatorInterface {
	private final Gemstick m_stickLeft, m_stickRight;
	private final XboxController m_controller;
	private final ClimberRollerListener m_rollerListener;

	public OperatorInterface(
			final int portLeft,
			final int portRight,
			final int portController
	) {
		m_stickLeft = new Gemstick(portLeft);
		m_stickRight = new Gemstick(portRight);
		m_controller = new XboxController(portController);

		final var hw = Hardware.getInstance();

		final Gembutton
				shiftDownButton = new Gembutton(m_stickLeft, 1),
				shiftUpButton = new Gembutton(m_stickRight, 1),
				intakeButton = new Gembutton(m_controller, 1),
				exhaustButton = new Gembutton(m_controller, 4),
				handButton = new Gembutton(m_controller, 5),
				armButton = new Gembutton(m_controller, 6),
				hab3FrontLegButton = new Gembutton(m_controller, 3),
				cargoHeightButton = new Gembutton(m_controller, 7),
				hab2FrontLegButton = new Gembutton(m_stickRight, 9),
				hab2BackLegButton = new Gembutton(m_stickRight, 11);

		final POVButton
				autoPickupButton = new POVButton(m_controller, POVState.W.toDegrees()),
				height1Button = new POVButton(m_controller, POVState.S.toDegrees()),
				height2Button = new POVButton(m_controller, POVState.E.toDegrees()),
				height3Button = new POVButton(m_controller, POVState.N.toDegrees());

		m_rollerListener = new ClimberRollerListener(hw.getRollers(), m_controller);

		final Trigger ptoDeployButton = new Trigger() {
			@Override
			public boolean get() {
				final var ds = DriverStation.getInstance();
				return (!ds.isAutonomous() || ds.getMatchTime() < 30) && m_stickRight.getRawButton(7);
			}
		};

		ptoDeployButton.whenActive(commandOf(() -> {
			hw.getPTO().engage();
			hw.getBackLegs().set(DoubleSolenoid.Value.kReverse);
			hw.getFrontLegs().set(true);
			hw.getManipulator().setSetSpeed(RunMode.HALTED);
			hw.getLateralAdjuster().disable();

			if (!m_rollerListener.hasPreviouslyRun()) {
				Scheduler.getInstance().add(m_rollerListener);
			}
		}));

		hab3FrontLegButton.whenInactive(commandOf(() -> {
			hw.getFrontLegs().set(false);
			hw.getPTO().disengage();
		}));

		hab2FrontLegButton.whenPressed(commandOf(() -> {
		  	  if (hw.getPTO().isEngaged()) {
				  hw.getFrontLegs().set(true);
				  hw.getPTO().disengage();
			  }
		}));

		hab2FrontLegButton.whenPressed(() ->
			  hw.getFrontLegs().set(true));

		hab2FrontLegButton.whenReleased(() ->
			  hw.getFrontLegs().set(false));

		hab2BackLegButton.whenPressed(() ->
		      hw.getBackLegs().set(DoubleSolenoid.Value.kReverse));

		hab2BackLegButton.whenReleased(() ->
			  hw.getBackLegs().set(DoubleSolenoid.Value.kForward));

		final Runnable intakeNeutralizer = () ->
			  hw.getManipulator().setSetSpeed(RunMode.NEUTRAL);

		final var transmission = hw.getChassis().getTransmission();
		final var manipulator = hw.getManipulator();
		final var inventory = hw.getInventory();
		final var lift = hw.getLift();

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

		handButton.whenPressed(() ->
			manipulator.getHand().set(true));
		handButton.whenReleased(() ->
			manipulator.getHand().set(false));
		armButton.whenPressed(() ->
			manipulator.getArm().set(true));
		armButton.whenReleased(() ->
		    manipulator.getArm().set(false));

		final var autoPickupSequence = new AutoPickupCommand(
				manipulator,
				Hardware.getInstance().getLimelight()
		);

		autoPickupButton.whenPressed(autoPickupSequence);
		autoPickupButton.whenReleased(commandOf(() -> {
			if (autoPickupSequence.isRunning()) {
				manipulator.getArm().set(false);
				manipulator.getHand().set(false);
			}

			autoPickupSequence.cancel();
		}));

		final var autoPlaceFactory = new AutoPlaceFactory(lift, manipulator);

		cargoHeightButton.whenPressed(commandOf(() ->
			lift.setPosition(Lift.Position.CARGO_SHIP)));

		height1Button.whenPressed(commandOf(() -> {
			Lift.Position position = null;

			switch (inventory.getCurrentPiece()) {
				case PANEL:
					position = Lift.Position.PANEL_1;
					System.out.println("Moved to PANEL_1");
					break;
				case CARGO:
					lift.setPosition(Lift.Position.CARGO_1);
					System.out.println("Moved to CARGO_1");
					return;
				case NONE:
				default:
					System.out.println("Setpoint commanded, but no gamepiece found!");
					break;
			}

			if (!Objects.isNull(position)) {
				if (m_controller.getRawButton(8)) {
					lift.setPosition(position);
				} else {
					Scheduler.getInstance().add(autoPlaceFactory.makeAutoPlace(position));
				}
			}
		}));

		height2Button.whenPressed(commandOf(() -> {
			Lift.Position position = null;

			switch (inventory.getCurrentPiece()) {
				case PANEL:
					position = Lift.Position.PANEL_2;
					System.out.println("Moved to PANEL_2");
					break;
				case CARGO:
					lift.setPosition(Lift.Position.CARGO_2);
					System.out.println("Moved to CARGO_2");
					return;
				case NONE:
				default:
					System.out.println("Setpoint commanded, but no gamepiece found!");
					break;
			}

			if (!Objects.isNull(position)) {
				if (m_controller.getRawButton(8)) {
					lift.setPosition(position);
				} else {
					Scheduler.getInstance().add(autoPlaceFactory.makeAutoPlace(position));
				}
			}
		}));

		height3Button.whenPressed(commandOf(() -> {
			Lift.Position position = null;

			switch (inventory.getCurrentPiece()) {
				case PANEL:
					position = Lift.Position.PANEL_3;
					System.out.println("Moved to PANEL_3");
					break;
				case CARGO:
					lift.setPosition(Lift.Position.CARGO_3);
					System.out.println("Moved to CARGO_3");
					return;
				case NONE:
				default:
					System.out.println("Setpoint commanded, but no gamepiece found!");
					break;
			}

			if (!Objects.isNull(position)) {
				if (m_controller.getRawButton(8)) {
					lift.setPosition(position);
				} else {
					Scheduler.getInstance().add(autoPlaceFactory.makeAutoPlace(position));
				}
			}
		}));
	}

	public void resetControls() {
		m_rollerListener.reset();
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
