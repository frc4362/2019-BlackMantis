package org.usfirst.frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.utility.control.RobotState;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveResetPoseFromPath extends InstantCommand {
    protected PathContainer pathContainer;
    protected boolean usePathRotation;
    protected RigidTransform2d transform;

    public DriveResetPoseFromPath(PathContainer pathContainer, boolean usePathRotation) {
        this.pathContainer = pathContainer;
        this.transform = null;
        this.usePathRotation = usePathRotation;
	}

    public DriveResetPoseFromPath(RigidTransform2d transform, boolean usePathRotation) {
        this.pathContainer = null;
        this.transform = transform;
        this.usePathRotation = usePathRotation;
	}

	@Override
	protected void initialize() {
        RigidTransform2d startPose = (transform == null) ? pathContainer.getStartPose() : transform;
		startPose.setRotation(RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation());
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
	}
}