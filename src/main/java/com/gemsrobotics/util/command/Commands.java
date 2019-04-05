package com.gemsrobotics.util.command;


import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * A utility class for the creation and composition of {@link Command}s
 * and {@link CommandGroup}s
 */
public final class Commands {
	private Commands() {}

	/**
	 * Turns an instance of {@link Runnable} into a one-time execution command.
	 * Reduces the amount of nice classes and {@link Command}s that need to be made
	 * @param action The {@link Runnable} to be turned into an {@link InstantCommand}
	 * @return The instantiated version of the passed {@link Runnable}
	 */
	public static InstantCommand commandOf(final Runnable action) {
		return new InstantCommand() {
			// FredBoat is with u
			// EDIT 10/26: ohhhhhh it's a joke about the word protected
			protected void initialize() {
				action.run();
			}
		};
	}

	/**
	 * Pass {@link Command} to be used to make a quick {@link CommandGroup}
	 * Generally reduces top-level clutter.
	 * @param actions The {@link Command}s to be made into a command group
	 */
	public static CommandGroup autonOf(final Command... actions) {
		return new CommandGroup() {
			{
				Arrays.asList(actions).forEach(this::addSequential);
			}
		};
	}

	// just a compatibility modifier
	public static CommandGroup autonOf(final List<Command> actions) {
		return autonOf(actions.toArray(new Command[0]));
	}

	/**
	 * @return A useless, do-nothing command for composition and a default value in various places
	 */
	public static InstantCommand nullCommand() {
		return commandOf(() -> {});
	}
}
