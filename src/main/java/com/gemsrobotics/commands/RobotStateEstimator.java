package com.gemsrobotics.commands;

import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.subsystems.drive.DifferentialDrive.Side;
import com.gemsrobotics.util.motion.Rotation;
import com.gemsrobotics.util.motion.Twist;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class RobotStateEstimator extends Command {
	public static final double dt = 0.02;

	private double m_encoderPrevDistanceLeft;
	private double m_encoderPrevDistanceRight;

	private DifferentialDrive m_vehicle;

	public RobotStateEstimator(final DifferentialDrive vehicle) {
		m_encoderPrevDistanceLeft = 0.0;
		m_encoderPrevDistanceRight = 0.0;

		m_vehicle = vehicle;
	}

	@Override
	public void initialize() {
		m_encoderPrevDistanceLeft = m_vehicle.getInchesPosition(Side.LEFT);
		m_encoderPrevDistanceRight = m_vehicle.getInchesPosition(Side.RIGHT);
	}
	int count = 0;
	@Override
	public void execute() {
		final double distanceLeft = m_vehicle.getInchesPosition(Side.LEFT),
				distanceRight = m_vehicle.getInchesPosition(Side.RIGHT);

		final Rotation angle = Rotation.fromDegrees(m_vehicle.getAHRS().getAngle());
		final Twist velocityMeasured = m_vehicle.getState().generateOdometry(
				distanceLeft - m_encoderPrevDistanceLeft,
				distanceRight - m_encoderPrevDistanceRight,
				angle);
		final Twist velocityPredicted = m_vehicle.getKinematics().forwardKinematics(
				m_vehicle.getInchesPerSecond(Side.LEFT) * dt,
				m_vehicle.getInchesPerSecond(Side.RIGHT) * dt);

//		if(count++ > 10) {
//			System.out.println("DrivePower Measured: " + velocityMeasured.toString());
//			System.out.println("DrivePower Predicted: " + velocityPredicted.toString());
//			count = 0;
//		}

		m_vehicle.getState().addObservations(
				Timer.getFPGATimestamp(),
				velocityMeasured,
				velocityPredicted);

		m_encoderPrevDistanceLeft = distanceLeft;
		m_encoderPrevDistanceRight = distanceRight;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
