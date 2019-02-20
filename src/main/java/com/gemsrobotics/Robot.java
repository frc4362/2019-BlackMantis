package com.gemsrobotics;

import com.gemsrobotics.commands.LiftScrubber;
import com.gemsrobotics.commands.LiftUnstucker;
import com.gemsrobotics.commands.ShifterListener;
import com.gemsrobotics.util.camera.Limelight.LEDMode;
import com.gemsrobotics.util.camera.Limelight.CameraMode;
import com.gemsrobotics.util.command.loggers.LimelightLogger;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// TODO look at what commands are required to be run full time
// see: everything in the commands dir
public class Robot extends TimedRobot {
	private OperatorInterface m_oi;
	private Hardware m_hardware;
	private SendableChooser<Boolean> m_compressorToggler, m_driveTrainToggler;

	@Override
	public void robotInit() {
		m_hardware = Hardware.getInstance();
		m_oi = new OperatorInterface(0, 1, 2);

		final var limelight = m_hardware.getLimelight();
		limelight.setCameraMode(CameraMode.CV);

		m_compressorToggler = new SendableChooser<>() {{
			setDefaultOption("Compressor OFF", false);
			addOption("Compressor ON", true);
		}};

		m_driveTrainToggler = new SendableChooser<>() {{
			setDefaultOption("Drive Train ON", true);
			addOption("Drive Train OFF", false);
		}};

		m_hardware.getChassis().configureDriveCommand(limelight, m_oi, m_driveTrainToggler);

		SmartDashboard.putData("Compressor", m_compressorToggler);
		SmartDashboard.putData("Drive", m_driveTrainToggler);
	}

	private void initDriverControl() {
		final var limelight = m_hardware.getLimelight();
		limelight.setLEDMode(LEDMode.ON);
		Scheduler.getInstance().add(new LimelightLogger(limelight));

		final var lift = m_hardware.getLift();
		lift.setIdleMode(CANSparkMax.IdleMode.kBrake);
		Scheduler.getInstance().add(new LiftScrubber(lift, m_oi.getController()));
		Scheduler.getInstance().add(new LiftUnstucker(lift));
		Scheduler.getInstance().add(lift.makeLogger());

		final var chassis = m_hardware.getChassis();
		Scheduler.getInstance().add(chassis.getStateEstimator());
		Scheduler.getInstance().add(chassis.getState().makeLogger());
		Scheduler.getInstance().add(chassis.getDriveCommand());

		Scheduler.getInstance().add(m_hardware.getInventory().makeLogger());
		Scheduler.getInstance().add(new ShifterListener(m_hardware.getLEDs(), chassis.getTransmission()));
	}

	@Override
	public void teleopInit() {
		Scheduler.getInstance().removeAll();
		initDriverControl();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		Scheduler.getInstance().removeAll();
		initDriverControl();
	}

	@Override
	public void robotPeriodic() {
		Hardware.getInstance()
				.getCompressor()
				.setClosedLoopControl(m_compressorToggler.getSelected());
	}

	@Override
	public void disabledInit() {
		m_hardware.getChassis().getMotors().forEach(motor ->
			motor.setIdleMode(CANSparkMax.IdleMode.kCoast));
		m_hardware.getLift().setIdleMode(CANSparkMax.IdleMode.kCoast);
		m_hardware.getLimelight().setLEDMode(LEDMode.OFF);
	}
}
