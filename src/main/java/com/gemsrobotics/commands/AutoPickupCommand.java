package com.gemsrobotics.commands;

import com.gemsrobotics.Config;
import com.gemsrobotics.commands.any.Wait;
import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.subsystems.drive.DriveCommandConfig;
import com.gemsrobotics.util.camera.Limelight;
import edu.wpi.first.wpilibj.command.Command;

import com.gemsrobotics.subsystems.manipulator.Manipulator;
import edu.wpi.first.wpilibj.command.Scheduler;

import java.util.Collections;

import static com.gemsrobotics.util.command.Commands.commandGroupOf;
import static com.gemsrobotics.util.command.Commands.commandOf;

public class AutoPickupCommand extends Command {
	private static final DifferentialDrive.DrivePower BACKWARDS_VELOCITY =
			DifferentialDrive.driveVelocity(-1.0, 0.0);
	private static final int BACKUP_TICKS = 20;

	private final Manipulator m_manipulator;
	private final Limelight m_limelight;
	private final DriveCommandConfig m_cfg;
	private final DifferentialDrive m_driveTrain;

	private State m_state;
	private boolean m_isFinished;
	private long m_lastOpenTime;

	public AutoPickupCommand(
			final Manipulator manipulator,
			final Limelight limelight,
			final DifferentialDrive chassis
	) {
		m_manipulator = manipulator;
		m_limelight = limelight;
		m_driveTrain = chassis;
		// we'll just carry this technical debt through the rest of the season
		m_cfg = Config.getConfig("driveCommand").to(DriveCommandConfig.class);
	}

	private enum State {
		APPROACHING, READY_TO_EXTEND, READY_FOR_PICKUP, READY_FOR_DEPARTURE
	}

	@Override
	public void initialize() {
		m_isFinished = false;
		m_state = State.APPROACHING;
		m_lastOpenTime = 0;
	}

	@Override
	public void execute() {
		final var area = m_limelight.getArea();

		if (area > m_cfg.slowdownCloseThreshold && m_state == State.APPROACHING) {
			m_state = State.READY_TO_EXTEND;
			m_manipulator.getHand().set(true);
			m_lastOpenTime = System.currentTimeMillis();
		}

		if (area > m_cfg.slowdownExtendThreshold && m_state == State.READY_TO_EXTEND) {
			m_state = State.READY_FOR_PICKUP;
			m_manipulator.getArm().set(true);
		}

		if (area > m_cfg.slowdownOpenThreshold && m_state == State.READY_FOR_PICKUP
			&& (System.currentTimeMillis() - m_lastOpenTime) > 200) {
			m_state = State.READY_FOR_DEPARTURE;
			// probably keep this here
			Scheduler.getInstance().add(makeBackupCommand());
			m_manipulator.getHand().set(false);
		}

		// if the driver backs out
		if (area < m_cfg.slowdownResetThreshold && m_state == State.READY_FOR_PICKUP) {
			m_state = State.APPROACHING;
			m_manipulator.getArm().set(false);
			m_manipulator.getHand().set(false);
		}

		if (area < m_cfg.slowdownPickupThreshold && m_state == State.READY_FOR_DEPARTURE) {
			m_isFinished = true;
			m_manipulator.getArm().set(false);
		}
	}

	private Command makeBackupCommand() {
		final var backupTrajectory = Collections.nCopies(BACKUP_TICKS, BACKWARDS_VELOCITY);
		return commandGroupOf(
				new Wait(50),
				commandOf(() -> m_driveTrain.getDriveCommand().queue(backupTrajectory)));
	}

	@Override
	public boolean isFinished() {
		return m_isFinished;
	}
}
