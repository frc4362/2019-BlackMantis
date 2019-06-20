package com.gemsrobotics.commands.auton;

import org.usfirst.frc.team3310.paths.PathBuilder;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.utility.control.Path;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Rotation2d;
import org.usfirst.frc.team3310.utility.math.Translation2d;

import java.util.List;

public class DriveForwardCurvePath implements PathContainer {
	@Override
	public Path buildPath() {
		return PathBuilder.buildPathFromWaypoints(List.of(
				new PathBuilder.Waypoint(Translation2d.identity(), 0, 0),
				new PathBuilder.Waypoint(new Translation2d(0, 36), 0, 60),
				new PathBuilder.Waypoint(new Translation2d(36, 72), 30, 60)
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
