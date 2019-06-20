package org.usfirst.frc.team3310.paths.auton;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team3310.paths.PathBuilder;
import org.usfirst.frc.team3310.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.utility.control.Path;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Rotation2d;
import org.usfirst.frc.team3310.utility.math.Translation2d;


public class GoStraight implements PathContainer {
    
    @Override
    public Path buildPath() {
        List<Waypoint> waypoints = new ArrayList<Waypoint>();
        waypoints.add(new Waypoint(18,70,0,0));
        waypoints.add(new Waypoint(150,70,0,60));

        return PathBuilder.buildPathFromWaypoints(waypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(18, 70), Rotation2d.fromDegrees(180)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
}