package com.gemsrobotics.commands;

import java.util.Objects;

import com.gemsrobotics.Hardware;
import com.gemsrobotics.subsystems.lift.Lift;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoPlaceCommand extends CommandGroup {
	private static double PLACEMENT_THRESHOLD = 25;

	private static AutoPlaceFactory autoPlaceFactory;

	private static AutoPlaceFactory getFactory() {
		if (Objects.isNull(autoPlaceFactory)) {
			autoPlaceFactory = new AutoPlaceFactory(
					Hardware.getInstance().getLift(),
					Hardware.getInstance().getManipulator());
		}

		return autoPlaceFactory;
	}

	public AutoPlaceCommand(final Lift.Position position) {
		addSequential(new WaitForAreaCommand(
				Hardware.getInstance().getLimelight(),
				PLACEMENT_THRESHOLD));
		addSequential(getFactory().makeAutoPlace(position));
	}
}
