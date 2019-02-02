package com.gemsrobotics.commands;

import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.util.motion.Rotation;
import com.gemsrobotics.util.motion.Twist;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import java.util.function.Supplier;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class RobotStateEstimator extends Command {
	public static final double dt = 0.02;

	private double m_encoderPrevDistanceLeft;
	private double m_encoderPrevDistanceRight;

	private DifferentialDrive m_chassis;
	private Supplier<Double> m_angleSupplier;

	public RobotStateEstimator(
			final DifferentialDrive driveTrain,
			final Supplier<Double> angleSupplier
	) {
		m_encoderPrevDistanceLeft = 0.0;
		m_encoderPrevDistanceRight = 0.0;

		m_chassis = driveTrain;
		m_angleSupplier = angleSupplier;
	}

	@Override
	public void initialize() {
		m_encoderPrevDistanceLeft = m_chassis.getInchesPosition(DifferentialDrive.Side.LEFT);
		m_encoderPrevDistanceRight = m_chassis.getInchesPosition(DifferentialDrive.Side.RIGHT);
	}

	@Override
	public void execute() {
		final double distanceLeft = m_chassis.getInchesPosition(DifferentialDrive.Side.LEFT),
				distanceRight = m_chassis.getInchesPosition(DifferentialDrive.Side.RIGHT);

		final Rotation angle = Rotation.fromDegrees(m_angleSupplier.get());
		final Twist velocityMeasured = m_chassis.getState().generateOdometry(
				distanceLeft - m_encoderPrevDistanceLeft,
				distanceRight - m_encoderPrevDistanceRight,
				angle);
		final Twist velocityPredicted = m_chassis.getKinematics().forwardKinematics(
				m_chassis.getInchesPerSecond(DifferentialDrive.Side.LEFT) * dt,
				m_chassis.getInchesPerSecond(DifferentialDrive.Side.RIGHT) * dt);

		m_chassis.getState().addObservations(
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
