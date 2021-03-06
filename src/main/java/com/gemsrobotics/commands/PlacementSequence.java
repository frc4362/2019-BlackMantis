package com.gemsrobotics.commands;

import com.gemsrobotics.commands.any.Wait;
import com.gemsrobotics.subsystems.manipulator.Manipulator;
import edu.wpi.first.wpilibj.command.CommandGroup;

import static com.gemsrobotics.util.command.Commands.commandOf;

@SuppressWarnings("WeakerAccess")
public class PlacementSequence extends CommandGroup {
	public PlacementSequence(
			final Manipulator manipulator,
			final boolean open
	) {
		addSequential(commandOf(() -> {
			manipulator.getArm().set(true);
			manipulator.getHand().set(false);
		}));
		addSequential(new Wait(200));
		addSequential(commandOf(() -> {
			manipulator.getArm().set(true);
			manipulator.getHand().set(true);
		}));
		addSequential(new Wait(250));
		addSequential(commandOf(() -> {
			manipulator.getArm().set(false);
			manipulator.getHand().set(true);
		}));
		addSequential(new Wait(100));
		addSequential(commandOf(() -> {
			manipulator.getArm().set(false);
			manipulator.getHand().set(!open);
		}));
	}
}
