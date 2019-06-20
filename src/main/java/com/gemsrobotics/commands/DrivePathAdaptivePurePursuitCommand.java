package com.gemsrobotics.commands;

import com.gemsrobotics.Constants;
import com.gemsrobotics.Hardware;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.utility.control.*;
import org.usfirst.frc.team3310.utility.math.Twist2d;

public class DrivePathAdaptivePurePursuitCommand extends Command {
	private final PathContainer m_pathContainer;
	private final Path m_path;

	private PathFollower m_follower;

	public DrivePathAdaptivePurePursuitCommand(final PathContainer pathContainer) {
		m_pathContainer = pathContainer;
		m_path = pathContainer.buildPath();
	}

	@Override
	public void initialize() {
		RobotState.getInstance().resetDistanceDriven();

		m_follower = new PathFollower(m_path, m_pathContainer.isReversed(),
				new PathFollower.Parameters(
						new Lookahead(Constants.kMinLookAhead,
									  Constants.kMaxLookAhead,
									  Constants.kMinLookAheadSpeed,
									  Constants.kMaxLookAheadSpeed),
						Constants.kInertiaSteeringGain,
						Constants.kPathFollowingProfileKp,
						Constants.kPathFollowingProfileKi,
						Constants.kPathFollowingProfileKv,
						Constants.kPathFollowingProfileKffv,
						Constants.kPathFollowingProfileKffa,
						Constants.kPathFollowingMaxVel,
						Constants.kPathFollowingMaxAccel,
						Constants.kPathFollowingGoalPosTolerance,
						Constants.kPathFollowingGoalVelTolerance,
						Constants.kPathStopSteeringDistance)
		);
	}

	@Override
	public void execute() {
		final double time = Timer.getFPGATimestamp();

		final var robotState = RobotState.getInstance();
		final var currentPose = robotState.getLatestFieldToVehicle().getValue();
		final double distanceDriven = robotState.getDistanceDriven();

		final Twist2d commandedChange = m_follower.update(time, currentPose, distanceDriven, robotState.getPredictedVelocity().dx);

		Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(commandedChange);

		Hardware.getInstance().getChassis().setVelocitySetpoints(setpoint.left, setpoint.right);
	}

	@Override
	public boolean isFinished() {
		return m_follower.isFinished();
	}
}
