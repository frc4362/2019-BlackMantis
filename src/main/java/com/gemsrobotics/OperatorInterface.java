package com.gemsrobotics;

import com.gemsrobotics.commands.AutoPickupCommand;
import com.gemsrobotics.commands.AutoPlaceFactory;
import com.gemsrobotics.commands.ClimberRollerListener;
import com.gemsrobotics.subsystems.inventory.Inventory;
import com.gemsrobotics.subsystems.lift.Lift;
import com.gemsrobotics.subsystems.manipulator.Manipulator;
import com.gemsrobotics.subsystems.manipulator.Manipulator.RunMode;
import com.gemsrobotics.util.DualTransmission.Gear;
import com.gemsrobotics.util.command.Commands;
import com.gemsrobotics.util.joy.Gembutton;
import com.gemsrobotics.util.joy.Gemstick;
import com.gemsrobotics.util.joy.Gemstick.POVState;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
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
	private final Lift m_lift;
	private final Manipulator m_manipulator;
	private final Inventory m_inventory;

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
		m_manipulator = hw.getManipulator();
		m_inventory = hw.getInventory();
		m_lift = hw.getLift();

		shiftUpButton.whenPressed(() ->
		    transmission.set(Gear.HIGH));
		shiftDownButton.whenPressed(() ->
		    transmission.set(Gear.LOW));

		intakeButton.whileHeld(() ->
		    m_manipulator.setSetSpeed(RunMode.INTAKING));
		intakeButton.whenReleased(intakeNeutralizer);

		exhaustButton.whileHeld(() ->
		    m_manipulator.setSetSpeed(RunMode.EXHAUSTING));
		exhaustButton.whenReleased(intakeNeutralizer);

		handButton.whenPressed(() ->
			m_manipulator.getHand().set(true));
		handButton.whenReleased(() ->
			m_manipulator.getHand().set(false));
		armButton.whenPressed(() ->
			m_manipulator.getArm().set(true));
		armButton.whenReleased(() ->
		    m_manipulator.getArm().set(false));

		final var autoPickupSequence = new AutoPickupCommand(
				m_manipulator,
				Hardware.getInstance().getLimelight()
		);

		autoPickupButton.whenPressed(autoPickupSequence);
		autoPickupButton.whenReleased(commandOf(() -> {
			if (autoPickupSequence.isRunning()) {
				m_manipulator.getArm().set(false);
				m_manipulator.getHand().set(false);
			}

			autoPickupSequence.cancel();
		}));

		final var autoPlaceFactory = new AutoPlaceFactory(m_lift, m_manipulator);

		cargoHeightButton.whenPressed(commandOf(() ->
			m_lift.setPosition(Lift.Position.CARGO_SHIP)));

		generatePlacementBindings(
				autoPlaceFactory,
				height1Button,
				Lift.Position.PANEL_1,
				Lift.Position.CARGO_1);

		generatePlacementBindings(
				autoPlaceFactory,
				height2Button,
				Lift.Position.PANEL_2,
				Lift.Position.CARGO_2);

		generatePlacementBindings(
				autoPlaceFactory,
				height3Button,
				Lift.Position.PANEL_3,
				Lift.Position.CARGO_3);
	}

	public void generatePlacementBindings(
			final AutoPlaceFactory autoPlaceFactory,
			final Button button,
			final Lift.Position panelPosition,
			final Lift.Position cargoPosition
	) {
		button.whenPressed(commandOf(() -> {
			switch (m_inventory.getCurrentPiece()) {
				case PANEL:
					if (m_controller.getRawButton(8)) {
						m_lift.setPosition(panelPosition);
					} else {
						final var cmd = autoPlaceFactory.makeAutoPlace(
								panelPosition,
								false);

						if (!Objects.isNull(cmd)) {
							Scheduler.getInstance().add(cmd);
							Scheduler.getInstance().add(Commands.listenForFinish(cmd,
									Commands.waitForRelease(
											new JoystickButton(m_controller, 8),
											() -> m_manipulator.getHand().set(false))));
						}
					}
					break;
				case CARGO:
					m_lift.setPosition(cargoPosition);
					break;
				case NONE:
				default:
					break;
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
