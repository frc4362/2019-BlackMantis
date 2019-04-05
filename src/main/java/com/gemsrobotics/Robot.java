package com.gemsrobotics;

import com.gemsrobotics.commands.*;
import com.gemsrobotics.util.DualTransmission;
import com.gemsrobotics.util.camera.Limelight.LEDMode;
import com.gemsrobotics.util.camera.Limelight.CameraMode;
import com.gemsrobotics.util.command.loggers.LimelightLogger;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	private OperatorInterface m_oi;
	private Hardware m_hardware;
	private SendableChooser<Boolean> m_compressorToggler;

	private boolean m_isFieldMatch;

	@Override
	public void robotInit() {
		m_isFieldMatch = false;
		m_oi = new OperatorInterface(0, 1, 2);

		m_hardware = Hardware.getInstance();

		m_compressorToggler = new SendableChooser<>() {{
			setDefaultOption("Compressor OFF", false);
			addOption("Compressor ON", true);
		}};

		final var driveTrainToggler = new SendableChooser<Boolean>() {{
			setDefaultOption("Drive Train ON", true);
			addOption("Drive Train OFF", false);
		}};

		SmartDashboard.putData("Compressor", m_compressorToggler);
		SmartDashboard.putData("Drive", driveTrainToggler);

		m_hardware.getChassis().getTransmission().set(DualTransmission.Gear.LOW);

		final var limelight = m_hardware.getLimelight();
		limelight.setCameraMode(CameraMode.CV);
		m_hardware.getChassis().configureDriveCommand(limelight, m_oi, driveTrainToggler);
	}

	private void initDriverControl() {
		m_isFieldMatch = m_ds.isFMSAttached();

		m_oi.resetControls();

		m_hardware.getPTO().disengage();
		m_hardware.getBackLegs().set(DoubleSolenoid.Value.kForward);
		m_hardware.getFrontLegs().set(false);

		final var controller = m_oi.getController();

		final var limelight = m_hardware.getLimelight();
		limelight.setLEDMode(LEDMode.ON);
		Scheduler.getInstance().add(new LimelightLogger(limelight));

		final var lift = m_hardware.getLift();
		lift.setIdleMode(IdleMode.kBrake);
		Scheduler.getInstance().add(new LiftScrubber(lift, controller));
		Scheduler.getInstance().add(new LiftUnstucker(lift));
		Scheduler.getInstance().add(lift.makeLogger());

		final var chassis = m_hardware.getChassis();
		chassis.getMotors().forEach(motor ->
			motor.setIdleMode(IdleMode.kBrake));

		Scheduler.getInstance().add(chassis.getStateEstimator());
		Scheduler.getInstance().add(chassis.getState().makeLogger());
		Scheduler.getInstance().add(chassis.getDriveCommand());

		final var latListen = new VisionAdjuster(
				m_hardware.getLateralAdjuster(),
				limelight,
				m_hardware.getInventory(),
				m_oi.getController());

		Scheduler.getInstance().add(m_hardware.getInventory().makeLogger());
		Scheduler.getInstance().add(new ShifterListener(m_hardware.getLEDs(), chassis.getTransmission()));
		Scheduler.getInstance().add(latListen);

		Scheduler.getInstance().add(new CargoHeightBoostListener(
				lift,
				m_hardware.getManipulator(),
				m_hardware.getInventory()));
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
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
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
			motor.setIdleMode(m_isFieldMatch ? IdleMode.kBrake : IdleMode.kCoast));
		m_hardware.getLift().setIdleMode(IdleMode.kCoast);
		m_hardware.getLimelight().setLEDMode(LEDMode.OFF);
	}
}
