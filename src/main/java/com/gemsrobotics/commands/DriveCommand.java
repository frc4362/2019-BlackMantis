package com.gemsrobotics.commands;

import com.gemsrobotics.OperatorInterface;
import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.subsystems.drive.DifferentialDrive.Side;
import com.gemsrobotics.util.MyAHRS;
import com.gemsrobotics.util.camera.Limelight;
import com.gemsrobotics.util.joy.Gemstick;
import com.gemsrobotics.util.joy.Gemstick.Lens;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

import static jaci.pathfinder.Pathfinder.boundHalfDegrees;
import static jaci.pathfinder.Pathfinder.d2r;
import static java.lang.Math.abs;

@SuppressWarnings({"unused", "WeakerAccess"})
public class DriveCommand extends Command {
	private static final double kTurn = 0.01;
	private static final double kP = 0.92;
	private static final double
			SLOWDOWN_PERCENT = 3,
			PLACEMENT_PERCENT = 6,
			MINIMUM_SPEED = 0.3;

	private final DifferentialDrive m_chassis;
	private final Gemstick m_stick, m_wheel;
	private final Limelight m_limelight;
	private final MyAHRS m_ahrs;
	private final SendableChooser<Boolean> m_toggler;

	private DistanceFollower m_followerLeft, m_followerRight;
	private boolean m_isFollowingTrajectory, m_isApproachingLast;

	public DriveCommand(
			final DifferentialDrive chassis,
			final Limelight limelight,
			final OperatorInterface oi,
			final SendableChooser<Boolean> toggler
	) {
		m_toggler = toggler;
		m_chassis = chassis;
		m_ahrs = chassis.getAHRS();
		m_stick = oi.getStickLeft();
		m_wheel = oi.getStickRight();
		m_limelight = limelight;

		m_isFollowingTrajectory = false;
		m_isApproachingLast = false;

		m_followerLeft = new DistanceFollower();
		m_followerRight = new DistanceFollower();

		final var trajectoryVars = m_chassis.getLocals().getPIDFVA();
		trajectoryVars.configure(m_followerLeft);
		trajectoryVars.configure(m_followerRight);
	}

	private void setTrajectories(
			final Trajectory leftTrajectory,
			final Trajectory rightTrajectory
	) {
		m_isFollowingTrajectory = true;
		m_followerLeft.setTrajectory(leftTrajectory);
		m_followerRight.setTrajectory(rightTrajectory);
	}

	public void driveTrajectories(final TankModifier trajs) {
		setTrajectories(trajs.getLeftTrajectory(), trajs.getRightTrajectory());
	}

	public void drivePoints(final Waypoint[] points) {
		final var localizations = m_chassis.getLocals();
		final var cfg = localizations.getTrajectoryConfig();
		final Trajectory path = Pathfinder.generate(points, cfg);
		final var splitPaths = new TankModifier(path).modify(localizations.width);
		driveTrajectories(splitPaths);
	}

	private boolean isAttemptingVision() {
		return m_wheel.getRawButton(4);
	}

	private double calculateLimelightAdjustment() {
		return m_limelight.getOffsetHorizontal() * kP;
	}

	private boolean shouldSlowDown() {
		return m_limelight.getArea() > SLOWDOWN_PERCENT;
	}

	public void setFollowingTrajectory(final boolean value) {
		m_isFollowingTrajectory = value;
	}

	@Override
	public void execute() {
		if (!m_toggler.getSelected()) {
			return;
		}

		if (m_isFollowingTrajectory) {
			followTrajectory();
		} else {
			final double rotationalPower;
			final boolean isQuickTurn;

			double linearPower = m_stick.get(Lens.Y);

			final boolean isApproaching = isAttemptingVision() && m_limelight.isTargetPresent();

			if (isApproaching) {
				isQuickTurn = false;

				// if we're going forwards
				if (linearPower > 0) {
					rotationalPower = calculateLimelightAdjustment();
				} else {
					rotationalPower = m_wheel.get(Lens.X);
				}

				if (shouldSlowDown()) {
					final double ratio = 1 - (m_limelight.getArea() / PLACEMENT_PERCENT);
					final double mag = abs(linearPower);
					final double newSpeed = Math.min(mag * ratio + MINIMUM_SPEED, mag);
					linearPower = Math.copySign(newSpeed, linearPower);
				}
			} else {
				isQuickTurn = m_wheel.getRawButton(3);
				rotationalPower =  m_wheel.get(Lens.X);
			}

			m_chassis.curvatureDrive(
					linearPower,
					rotationalPower,
					isQuickTurn
			);

			m_isApproachingLast = isApproaching;
		}
	}

	private void followTrajectory() {
		final double heading = m_ahrs.getYaw(),
				desiredHeading = d2r(m_followerLeft.getHeading()),
				angleDifference = boundHalfDegrees(desiredHeading - heading),
				turn = angleDifference * kTurn;
		final double inchesDrivenLeft = m_chassis.getInchesPerSecond(Side.LEFT),
				leftVelocity = m_followerLeft.calculate(inchesDrivenLeft);
		final double inchesDrivenRight = m_chassis.getInchesPerSecond(Side.RIGHT),
				rightVelocity = -m_followerRight.calculate(inchesDrivenRight);

		m_chassis.drive(leftVelocity - turn, rightVelocity + turn);
		m_isFollowingTrajectory = leftVelocity != 0.0 || rightVelocity != 0.0;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
