package com.gemsrobotics.commands;

import com.gemsrobotics.Constants;
import com.gemsrobotics.Hardware;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.utility.control.*;
import org.usfirst.frc.team3310.utility.math.Twist2d;

import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.OpenOption;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.nio.file.attribute.FileAttribute;

public class DrivePathAdaptivePurePursuitCommand extends Command {
	private static final java.nio.file.Path DEBUG_PATH =
			Paths.get("/home/lvuser/profile_debug.csv");
	private static final String DEBUG_SCHEMA =
			"t,pose_x,pose_y,pose_theta,linear_displacement,linear_velocity,profile_displacement,profile_velocity,velocity_command_dx,velocity_command_dy,velocity_command_dtheta,steering_command_dx,steering_command_dy,steering_command_dtheta,cross_track_error,along_track_error,lookahead_point_x,lookahead_point_y,lookahead_point_velocity";

	private final PathContainer m_pathContainer;
	private final Path m_path;

	private PathFollower m_follower;

	private boolean m_doLogging;

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

		try {
			Files.deleteIfExists(DEBUG_PATH);
			Files.write(DEBUG_PATH, DEBUG_SCHEMA.getBytes(), StandardOpenOption.CREATE);
			m_doLogging = true;
		} catch (final IOException ioex) {
			ioex.printStackTrace();
			m_doLogging = false;
		}
	}

	@Override
	public void execute() {
		final double time = Timer.getFPGATimestamp();

		final var robotState = RobotState.getInstance();
		final var currentPose = robotState.getLatestFieldToVehicle().getValue();
		final double distanceDriven = robotState.getDistanceDriven();

		final Twist2d commandedChange = m_follower.update(time, currentPose, distanceDriven, robotState.getPredictedVelocity().dx);

		if (m_doLogging) {
			try {
				Files.write(
						DEBUG_PATH,
						(m_follower.getDebug().toString()
								 + "\"" + commandedChange.dx + "\",\""
								 + commandedChange.dy + "\",\""
								 + commandedChange.dtheta + "\"").getBytes(),
						StandardOpenOption.APPEND);
			} catch (final IOException e) {
				e.printStackTrace();
				cancel();
			}
		}

		Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(commandedChange);

		Hardware.getInstance().getChassis().setVelocityReferences(setpoint.left, setpoint.right);
	}

	@Override
	public boolean isFinished() {
		return m_follower.isFinished();
	}
}
