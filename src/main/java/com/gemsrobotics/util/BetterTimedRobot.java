package com.gemsrobotics.util;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.gemsrobotics.commands.any.Wait;
import com.gemsrobotics.util.command.Commands;
import com.gemsrobotics.util.command.DelayedAutonFactory;
import com.gemsrobotics.util.command.RuntimeCommandGroup;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.gemsrobotics.util.joy.Gemstick;

import static com.gemsrobotics.util.command.Commands.commandGroupOf;

/**
 * Wrapper class that works with a subclass to provide important functionality
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public abstract class BetterTimedRobot extends TimedRobot {
	private final boolean m_useDelayer;

	protected final XboxController m_controller;
	protected final Gemstick m_leftStick, m_rightStick;
	protected SendableChooser<Command> m_autonSelector;

	/**
	 * @param useDelayer Enables the delayer and after-routine safety command
	 */
	public BetterTimedRobot(final boolean useDelayer) {
		resetAutonSelector();

		m_useDelayer = useDelayer;

		m_leftStick  = new Gemstick("Left Stick", 0);
		m_rightStick = new Gemstick("Right Stick", 1);
		m_controller = new XboxController(2);
	}

	/**
	 * @return Should provide the commands to be run at the start of teleop
	 */
	public abstract List<Command> getTeleopCommands();

	/**
	 * @return Should provide the commands to be run at the start of auton
	 */
	public abstract SendableChooser<Command> getAutonCommands();
	public abstract List<Sendable> getSendables();

	/**
	 * Meant to be overridden.
	 * @return The auton that will run if all is lost.
	 */
	public Command getDefaultAuton() {
		return Commands.nullCommand();
	}

	/**
	 * Runs at inconsistent times
	 * Either when code starts up, when you enable, or when you connect, depending.
	 */
	@Override
	public void robotInit() {
		resetAutonSelector();
		getSendables().forEach(SmartDashboard::putData);
	}

	/**
	 * Meant to be overridden
	 */
	public void teleopStart() {
		System.out.println("No implementation of teleopStart... Override me!");
	}

	/**
	 * Meant to be overridden
	 */
	public void autonomousStart() {
		System.out.println("No implementation of autonomousStart... Override me!");
	}

	/**
	 * Manages the delay timer, which will add an optional delay,
	 * and prevent aberrant behavior after auton
	 */
	@Override
	public final void autonomousInit() {
		System.out.println("Entering auton...");
		final Command autonCommand = getSelectedAuton(this::getDefaultAuton);

		Scheduler.getInstance().removeAll();
		// only use the fancy stuff if you want delays
		if (m_useDelayer) {
			Scheduler.getInstance().add(
				new Wait(DelayedAutonFactory.getAppropriateDelay()) {
					@Override
					public void end() {
						final Command auton = Commands.commandGroupOf(autonCommand, new Wait(15000));
						Scheduler.getInstance().add(auton);
					}
				}
			);
		} else {
			Scheduler.getInstance().add(autonCommand);
		}

		autonomousStart();
	}

	/**
	 * Initializes the scheduler and stuff for teleop,
	 * adds {@link BetterTimedRobot#getTeleopCommands()}
	 */
	@Override
	public final void teleopInit() {
		Scheduler.getInstance().removeAll();
		getTeleopCommands().forEach(Scheduler.getInstance()::add);
		teleopStart();
	}

	/**
	 * @return Auton command returned by the SmartDashboard, or a default auton if none.
	 */
	public final Command getSelectedAuton(final Supplier<Command> defaultSupplier) {
		final Command auton =
				Optional.ofNullable(m_autonSelector.getSelected())
					    .orElseGet(defaultSupplier);

		// initializes commands which need it
		if (auton instanceof RuntimeCommandGroup) {
			((RuntimeCommandGroup) auton).init();
		}

		return auton;
	}

	/**
	 * Make a new one every time, helps prevent ghost-selectors
	 */
	protected void resetAutonSelector() {
		m_autonSelector = getAutonCommands();
		SmartDashboard.putData("auton", m_autonSelector);
	}
}
