package com.gemsrobotics.commands;

import com.gemsrobotics.commands.any.Wait;
import com.gemsrobotics.subsystems.manipulator.Manipulator;
import edu.wpi.first.wpilibj.command.CommandGroup;

import static com.gemsrobotics.util.command.Commands.commandOf;

public class PickupSequence extends CommandGroup {
	public PickupSequence(final Manipulator manipulator) {
		addSequential(commandOf(() -> {
			manipulator.getArm().set(true);
			manipulator.getHand().set(true);
		}));
		addSequential(new Wait(200));
		addSequential(commandOf(() -> {
			manipulator.getArm().set(true);
			manipulator.getHand().set(false);
		}));
		addSequential(new Wait(200));
		addSequential(commandOf(() -> {
			manipulator.getArm().set(false);
			manipulator.getHand().set(false);
		}));
	}
}
