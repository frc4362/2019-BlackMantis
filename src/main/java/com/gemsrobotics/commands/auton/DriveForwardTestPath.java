package com.gemsrobotics.commands.auton;

import org.usfirst.frc.team3310.paths.PathBuilder;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.utility.control.Path;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Translation2d;

import java.util.List;

public class DriveForwardTestPath implements PathContainer {
	@Override
	public Path buildPath() {
		return PathBuilder.buildPathFromWaypoints(List.of(
				new PathBuilder.Waypoint(new Translation2d(0, 0), 0, 0),
				new PathBuilder.Waypoint(new Translation2d(60, 0), 0, 0)
		));
	}

	@Override
	public RigidTransform2d getStartPose() {
		return RigidTransform2d.identity();
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}
