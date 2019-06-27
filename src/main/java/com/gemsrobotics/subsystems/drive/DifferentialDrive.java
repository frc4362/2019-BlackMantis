package com.gemsrobotics.subsystems.drive;

import com.gemsrobotics.OperatorInterface;
import com.gemsrobotics.commands.OpenLoopDriveCommand;
import com.gemsrobotics.commands.ShiftScheduler;
import com.gemsrobotics.util.DualTransmission;
import com.gemsrobotics.util.MyAHRS;
import com.gemsrobotics.util.PIDF;
import com.gemsrobotics.util.camera.Limelight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.usfirst.frc.team3310.utility.control.RobotStateEstimator;
import org.usfirst.frc.team3310.utility.math.Rotation2d;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static java.lang.Math.*;
import static java.lang.Math.min;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class DifferentialDrive extends Subsystem implements Sendable {
	public static final double dt = 0.02;

	private static final double DEGREES_THRESHOLD = 3.5;

	private final Specifications m_specifications;
	private final DualTransmission m_transmission;
	private final List<CANSparkMax> m_motors;
	private final MyAHRS m_ahrs;
	private final ShiftScheduler m_shiftScheduler;

	private OpenLoopDriveCommand m_driveCommand;
	private double m_accumulator;
	private String m_name;

	private static final String[] MOTOR_CONTROLLER_NAMES = {
		"Left Front", "Left Back", "Right Front", "Right Back"
	};

	public DifferentialDrive(
			final DrivePorts drivePorts,
			final Specifications specifications,
			final Solenoid shifter,
			final MyAHRS ahrs
	) {
		m_specifications = specifications;
		m_ahrs = ahrs;
		m_transmission = new DualTransmission(shifter);
		m_shiftScheduler = new ShiftScheduler(this);

		final Integer[] ports = drivePorts.get();

		if (ports.length != 4) {
			throw new RuntimeException("Wrong number of ports!");
		}

		m_motors = Arrays.stream(ports)
				.map(port -> new CANSparkMax(port, MotorType.kBrushless))
				.collect(Collectors.toList());

		m_motors.forEach(m -> {
			m.setIdleMode(CANSparkMax.IdleMode.kBrake);
			m.setInverted(false);

			m.getEncoder().setVelocityConversionFactor(m_specifications.encoderFactor);
			m.getEncoder().setPositionConversionFactor(m_specifications.encoderFactor * 60);

			final var pidController = m.getPIDController();
			m_specifications.pidDrive.configure(pidController, 0);
			pidController.setOutputRange(-1.0, +1.0);
		});

		m_motors.get(1).follow(m_motors.get(0));
		m_motors.get(3).follow(m_motors.get(2));

		m_accumulator = 0.0;
	}

	public void configureDriveCommand(
			final Limelight limelight,
			final OperatorInterface controls,
			final SendableChooser<Boolean> toggler
	) {
		m_driveCommand = new OpenLoopDriveCommand(this, limelight, controls, toggler);
	}

	final double MAGIC_ENCODER_NUMBER = 1.3631;

	public void setVelocityReferences(final double setpointLeft, final double setpointRight) {
		final double maxDesiredVelocity = max(abs(setpointLeft), abs(setpointRight));
		final double maxSetpointVelocity = m_transmission.get().topSpeed; // magic numbeeeeer
		final double scale = maxDesiredVelocity > maxSetpointVelocity ? maxSetpointVelocity / maxDesiredVelocity : 1.0;

		getMotor(Side.LEFT).getPIDController().setReference(setpointLeft * scale * MAGIC_ENCODER_NUMBER, ControlType.kVelocity, 0);
		getMotor(Side.RIGHT).getPIDController().setReference(-setpointRight * scale * MAGIC_ENCODER_NUMBER, ControlType.kVelocity, 0);
	}

	public void drive(final double leftPower, final double rightPower) {
		getMotor(Side.LEFT).set(leftPower);
		getMotor(Side.RIGHT).set(-rightPower);
	}

	private static final double LIMIT = 1.0;

	private static double limit(final double v) {
		return abs(v) < LIMIT ? v : LIMIT * signum(v);
	}

	private static double constrain(final double bot, final double val, final double top) {
		return max(bot, min(val, top));
	}

	public void curvatureDrive(
			final double linearPower,
			double zRotation,
			final boolean isQuickTurn
	) {
		double overPower, angularPower;

		if (isQuickTurn) {
			if (Math.abs(linearPower) < m_specifications.quickstopThreshold) {
				m_accumulator = (1 - m_specifications.alpha) * m_accumulator + m_specifications.alpha * limit(zRotation) * 2;
			}

			overPower = 1.0;
			angularPower = -zRotation;
		} else {
			overPower = 0.0;
			zRotation *= -signum(linearPower);
			angularPower = abs(linearPower) * zRotation * m_specifications.turnSensitivity - m_accumulator;

			if (m_accumulator > 1) {
				m_accumulator -= 1;
			} else if (m_accumulator < -1) {
				m_accumulator += 1;
			} else {
				m_accumulator = 0.0;
			}
		}

		double leftPower = linearPower - angularPower,
				rightPower = linearPower + angularPower;

		if (leftPower > 1.0) {
			rightPower -= overPower * (leftPower - 1.0);
			leftPower = 1.0;
		} else if (rightPower > 1.0) {
			leftPower -= overPower * (rightPower - 1.0);
			rightPower = 1.0;
		} else if (leftPower < -1.0) {
			rightPower += overPower * (-1.0 - leftPower);
			leftPower = -1.0;
		} else if (rightPower < -1.0) {
			leftPower += overPower * (-1.0 - rightPower);
			rightPower = -1.0;
		}

		drive(leftPower, rightPower);
	}

	public boolean turnToHeading(final double goal) {
		final var currentAngle = m_ahrs.getHalfAngle();

		double error = -(currentAngle - goal);

		if (abs(error) > 180) {
			error = 360 - error;

			if (currentAngle < 0 && goal > 0) {
				error *= -1;
			}
		}

		final boolean isAtHeading = abs(error) < DEGREES_THRESHOLD;

		if (!isAtHeading) {
			final double angularPower = error * m_specifications.kP_Rotational + copySign(m_specifications.kFF_Rotational, error);
			curvatureDrive(0, constrain(-1, angularPower, 1), true);
		}

		return isAtHeading;
	}

	public CANSparkMax getMotor(final Side side) {
		return m_motors.get(side.idx);
	}

	public double getInchesPerSecond(final Side side) {
//		final var rps = getMotor(side).getEncoder().getVelocity() / 60.0;
		return getMotor(side).getEncoder().getVelocity() * side.encoderMultiplier;
	}

	public double getInchesPosition(final Side side) {
		return getMotor(side).getEncoder().getPosition() * side.encoderMultiplier;
	}

	public void stopMotors() {
		m_motors.forEach(CANSparkMax::stopMotor);
	}

	public List<CANSparkMax> getMotors() {
		return m_motors;
	}

	public Specifications getLocals() {
		return m_specifications;
	}

	public DualTransmission getTransmission() {
		return m_transmission;
	}

	public Rotation2d getRotation() {
		return Rotation2d.fromDegrees(-m_ahrs.getAngle());
	}

	public OpenLoopDriveCommand getDriveCommand() {
		return m_driveCommand;
	}

	public RobotStateEstimator getStateEstimator() {
		return RobotStateEstimator.getInstance();
	}

	public ShiftScheduler getShiftScheduler() {
		return m_shiftScheduler;
	}

	public enum Side {
		LEFT(0, 1), RIGHT(2, -1);

		protected final int idx, encoderMultiplier;

		Side(final int i, final int mult) {
			idx = i;
			encoderMultiplier = mult;
		}

		private static Side forIndex(final int i) {
			if (i < 2) {
				return LEFT;
			} else {
				return RIGHT;
			}
		}
	}

	public static class Specifications {
		public double quickstopThreshold,
				turnSensitivity, alpha,
				kP_Rotational, kFF_Rotational, encoderFactor;

		public PIDF pidDrive;
	}

	@Override
	public void initDefaultCommand() { }

	@Override
	public void initSendable(final SendableBuilder builder) {
		builder.setSmartDashboardType("West Coast Drive");

		final List<Runnable> updaters = IntStream.range(0, 4).<Runnable>mapToObj(i -> {
			final CANSparkMax spark = m_motors.get(i);
			final String name = MOTOR_CONTROLLER_NAMES[i];

			final NetworkTableEntry
					setSpeedEntry = builder.getEntry(name + " Setpoint"),
					velocityEntry = builder.getEntry(name + " Velocity (In-s)"),
					positionEntry = builder.getEntry(name + " Position (In)");

			final String id = Integer.toString(spark.getDeviceId());
			builder.getEntry(name + " CAN ID").setString(id);

			return () -> {
				setSpeedEntry.setDouble(spark.get());
				velocityEntry.setDouble(getInchesPerSecond(Side.forIndex(i)));
				positionEntry.setDouble(getInchesPosition(Side.forIndex(i)));
			};
		}).collect(Collectors.toList());

		builder.setUpdateTable(() ->
			  updaters.forEach(Runnable::run));
	}
}
