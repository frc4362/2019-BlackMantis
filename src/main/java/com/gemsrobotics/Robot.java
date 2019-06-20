package com.gemsrobotics;

import com.gemsrobotics.commands.*;
import com.gemsrobotics.commands.auton.AutonomousCommandGroup;
import com.gemsrobotics.commands.auton.DriveForwardCurvePath;
import com.gemsrobotics.util.DualTransmission.Gear;
import com.gemsrobotics.util.camera.Limelight.LEDMode;
import com.gemsrobotics.util.camera.Limelight.CameraMode;
import com.gemsrobotics.util.command.Commands;
import com.gemsrobotics.util.command.loggers.LimelightLogger;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3310.utility.control.RobotState;
import org.usfirst.frc.team3310.utility.control.RobotStateEstimator;

public class Robot extends TimedRobot {
	private OperatorInterface m_oi;
	private Hardware m_hardware;
	private LEDController m_ledController;
	private SendableChooser<Boolean> m_compressorToggler, m_slideToggler;
	private SendableChooser<Command> m_autonSelector;

	private Command m_selectedAuton;
	private boolean m_hasAutonFinished, m_isFieldMatch;

	@Override
	public void robotInit() {
		m_isFieldMatch = false;
		m_oi = new OperatorInterface(0, 1, 2);

		m_hardware = Hardware.getInstance();

		m_compressorToggler = new SendableChooser<>() {{
			setDefaultOption("Compressor OFF", false);
			addOption("Compressor ON", true);
		}};

		m_slideToggler = new SendableChooser<>() {{
			setDefaultOption("Linear Slide ON", true);
			addOption("Linear Slide OFF", false);
		}};

		m_autonSelector = new SendableChooser<>() {{
			setDefaultOption("NONE", Commands.nullCommand());
			addOption("test auton", new AutonomousCommandGroup(m_oi) {{
				addSequential(new DrivePathAdaptivePurePursuitCommand(new DriveForwardCurvePath()));
			}});
		}};

		final var driveTrainToggler = new SendableChooser<Boolean>() {{
			setDefaultOption("Drive Train ON", true);
			addOption("Drive Train OFF", false);
		}};

		SmartDashboard.putData("Compressor", m_compressorToggler);
		SmartDashboard.putData("Drive", driveTrainToggler);
		SmartDashboard.putData("Linear Slide", m_slideToggler);
		SmartDashboard.putData("Auton Selector", m_autonSelector);

		final var limelight = m_hardware.getLimelight();
		limelight.setCameraMode(CameraMode.CV);

		final var chassis = m_hardware.getChassis();
		chassis.getTransmission().set(Gear.LOW);
		chassis.configureDriveCommand(limelight, m_oi, driveTrainToggler);

		m_ledController = new LEDController(m_oi.getController());

		m_hardware.getAHRS().reset();
	}

	@Override
	public void robotPeriodic() {
		m_ledController.writePeriodicOutputs();

		m_hardware.getCompressor().setClosedLoopControl(m_compressorToggler.getSelected());

		SmartDashboard.putBoolean("HIGH GEAR", m_hardware.getChassis().getTransmission().get() == Gear.HIGH);
		SmartDashboard.putBoolean("hand closed", m_hardware.getManipulator().getHand().get());
		SmartDashboard.putNumber("Yaw", m_hardware.getAHRS().getYaw());

		final var currentPosition = RobotState.getInstance().getLatestFieldToVehicle().getValue();
		SmartDashboard.putString("Robot State", currentPosition.toString());
	}

	private void initOpMode() {
		m_isFieldMatch = m_ds.isFMSAttached();

		m_oi.resetControls();

		m_hardware.getPTO().disengage();
		m_hardware.getBackLegs().set(DoubleSolenoid.Value.kForward);
		m_hardware.getStage1Solenoid().set(false);

		final var limelight = m_hardware.getLimelight();
		limelight.setLEDMode(LEDMode.ON);
		Scheduler.getInstance().add(new LimelightLogger(limelight));

		final var controller = m_oi.getController();

		final var lift = m_hardware.getLift();
		lift.setIdleMode(IdleMode.kBrake);
		Scheduler.getInstance().add(new LiftScrubber(lift, controller));
		Scheduler.getInstance().add(new LiftUnstucker(lift));
		Scheduler.getInstance().add(lift.makeLogger());

		Scheduler.getInstance().add(RobotStateEstimator.getInstance());

		final var chassis = m_hardware.getChassis();
		chassis.getMotors().forEach(motor ->
			motor.setIdleMode(IdleMode.kBrake));

		if (m_slideToggler.getSelected()) {
			Scheduler.getInstance().add(new VisionAdjuster(
					m_hardware.getLateralAdjuster(),
					limelight,
					m_hardware.getInventory(),
					m_oi.getController(),
					m_hardware.getLift()));
		}
	}

	private void initDriverControl() {
		Scheduler.getInstance().add(Hardware.getInstance().getChassis().getDriveCommand());
		Scheduler.getInstance().add(new CargoHeightBoostListener(
				Hardware.getInstance().getLift(),
				m_hardware.getManipulator(),
				m_hardware.getInventory()));
	}

	@Override
	public void autonomousInit() {
		Scheduler.getInstance().removeAll();
		initOpMode();

		m_selectedAuton = m_autonSelector.getSelected();
		m_hasAutonFinished = false;
		Scheduler.getInstance().add(m_selectedAuton);
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();

		if (m_selectedAuton.isCompleted() && !m_hasAutonFinished) {
			m_hasAutonFinished = true;
			initDriverControl();
		}
	}

	@Override
	public void teleopInit() {
		Scheduler.getInstance().removeAll();
		initOpMode();
		initDriverControl();
		m_hardware.getLimelight().setLEDMode(LEDMode.ON);
		Scheduler.getInstance().add(m_hardware.getChassis().getShiftScheduler());
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		final var idleMode = m_isFieldMatch ? IdleMode.kBrake : IdleMode.kCoast;
		m_hardware.getChassis().getMotors().forEach(motor ->
			motor.setIdleMode(idleMode));
		m_hardware.getLift().setIdleMode(idleMode);
		m_hardware.getLimelight().setLEDMode(LEDMode.OFF);
	}
}
