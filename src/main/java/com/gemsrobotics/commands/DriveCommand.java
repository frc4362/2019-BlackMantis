package com.gemsrobotics.commands;

import com.gemsrobotics.Config;
import com.gemsrobotics.OperatorInterface;
import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.subsystems.drive.DriveCommandConfig;
import com.gemsrobotics.util.DualTransmission;
import com.gemsrobotics.util.camera.Limelight;
import com.gemsrobotics.util.joy.Gemstick;
import com.gemsrobotics.util.joy.Gemstick.Lens;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Collection;
import java.util.LinkedList;
import java.util.Queue;

import static java.lang.Math.max;
import static java.lang.Math.min;

@SuppressWarnings({"unused", "WeakerAccess"})
public class DriveCommand extends Command {
	private static final double kP = 1.3;
	private static final double SLOWDOWN_PERCENT = 0.5;

	private final DifferentialDrive m_chassis;
	private final Gemstick m_stick, m_wheel;
	private final Limelight m_limelight;
	private final SendableChooser<Boolean> m_toggler;
	private final DriveCommandConfig m_cfg;

	private Queue<DifferentialDrive.DrivePower> m_driveQueue;

	private boolean m_isApproachingLast;

	public DriveCommand(
			final DifferentialDrive chassis,
			final Limelight limelight,
			final OperatorInterface oi,
			final SendableChooser<Boolean> toggler
	) {
		m_toggler = toggler;
		m_chassis = chassis;
		m_stick = oi.getStickLeft();
		m_wheel = oi.getStickRight();
		m_limelight = limelight;

		m_cfg = Config.getConfig("driveCommand").to(DriveCommandConfig.class);
	}

	private boolean isAttemptingVision() {
		return m_wheel.getRawButton(4);
	}

	private double calculateLimelightAdjustment() {
		return m_limelight.getOffsetHorizontal() * kP;
	}

	private boolean isSlowableArea() {
		return m_limelight.getArea() > SLOWDOWN_PERCENT;
	}

	public void queue(final Collection<DifferentialDrive.DrivePower> velocities) {
		m_driveQueue.addAll(velocities);
	}

	@Override
	public void initialize() {
		m_driveQueue = new LinkedList<>();
	}

	@Override
	public void execute() {
		if (!m_toggler.getSelected()) {
			return;
		}

		if (!m_driveQueue.isEmpty()) {
			m_chassis.drive(m_driveQueue.poll());
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
			} else {
				isQuickTurn = m_wheel.getRawButton(3);
				rotationalPower =  m_wheel.get(Lens.X);
			}

			if (DriverStation.getInstance().isAutonomous() || (isAttemptingVision() && isSlowableArea())) {
				final var previousPower = linearPower;
				final var area = m_limelight.getArea();

				if (area > m_cfg.stopOverdriveThreshold) {
					linearPower = min(0, previousPower);
				} else {
					if (m_chassis.getTransmission().get() == DualTransmission.Gear.HIGH) {
						final double highStartArea = 0.4;
						final double highEndArea = 8;
						final var scalingFactor = max(((area - highEndArea) / (highStartArea - highEndArea)), 0);
						linearPower = min(m_cfg.getRampingRange() * scalingFactor + 0, previousPower);
					} else {
						final var scalingFactor = max(((area - m_cfg.endRampArea) / m_cfg.getDivisor()), 0);
						linearPower = min(m_cfg.getRampingRange() * scalingFactor + m_cfg.minSpeed, previousPower);
					}
				}
			}

			m_chassis.curvatureDrive(
					linearPower,
					rotationalPower,
					isQuickTurn
			);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
