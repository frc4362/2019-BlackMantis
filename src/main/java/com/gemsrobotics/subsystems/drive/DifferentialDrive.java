package com.gemsrobotics.subsystems.drive;

import com.gemsrobotics.OperatorInterface;
import com.gemsrobotics.commands.DriveCommand;
import com.gemsrobotics.commands.RobotStateEstimator;
import com.gemsrobotics.util.DualTransmission;
import com.gemsrobotics.util.MyAHRS;
import com.gemsrobotics.util.PIDF;
import com.gemsrobotics.util.camera.Limelight;
import com.gemsrobotics.util.command.Commands;
import com.gemsrobotics.util.motion.Pose;
import com.gemsrobotics.util.motion.Rotation;
import com.gemsrobotics.util.motion.Twist;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import static com.gemsrobotics.util.motion.EpsilonValue.Epsilon;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class DifferentialDrive extends Subsystem implements Sendable {
	public static final double dt = 0.02;

	private static final double METER_TO_INCHES = 0.025399986284007407;

	private final Kinematics m_kinematics;
	private final Localizations m_localizations;
	private final DualTransmission m_transmission;
	private final RobotState m_state;
	private final List<CANSparkMax> m_motors;
	private final MyAHRS m_ahrs;

	private DriveCommand m_driveCommand;
	private boolean m_useVelocityControl;
	private double m_accumulator;
	private String m_name;

	private static final String[] SPARK_NAMES = {
		"Left Front", "Left Back", "Right Front", "Right Back"
	};

	private RobotStateEstimator m_robotStateEstimator;

	public DifferentialDrive(
			final DrivePorts drivePorts,
			final Localizations localizations,
			final DoubleSolenoid shifter,
			final MyAHRS ahrs,
			final boolean useVelocityControl
	) {
		final Integer[] ports = drivePorts.get();
		m_ahrs = ahrs;
		m_localizations = localizations;
		m_transmission = new DualTransmission(shifter);
		m_kinematics = new Kinematics(m_localizations, this);
		m_state = new RobotState(this);
		m_robotStateEstimator = new RobotStateEstimator(this);

		if (ports.length != 4) {
			throw new RuntimeException("Wrong number of ports!");
		}

		m_motors = Arrays.stream(ports)
				.map(port -> new CANSparkMax(port, MotorType.kBrushless))
				.collect(Collectors.toList());

		m_motors.get(1).follow(m_motors.get(0));
		m_motors.get(3).follow(m_motors.get(2));
		m_motors.forEach(m -> {
			m.setIdleMode(CANSparkMax.IdleMode.kBrake);
			m.setInverted(false);
		});

		m_motors.stream().map(CANSparkMax::getPIDController).forEach(controller -> {
			m_localizations.pidDrive.configure(controller);
			controller.setOutputRange(-1.0, +1.0);
		});

		m_useVelocityControl = useVelocityControl;
		m_accumulator = 0.0;
	}

	public void configureDriveCommand(
			final Limelight limelight,
			final OperatorInterface controls,
			final SendableChooser<Boolean> toggler
	) {
		m_driveCommand = new DriveCommand(this, limelight, controls, toggler);
	}

	public void drive(final double leftPower, final double rightPower) {
		SmartDashboard.putString("drive state", "drive started");
		final var leftMotor = getMotor(Side.LEFT);
		final var rightMotor = getMotor(Side.RIGHT);

		if (m_useVelocityControl) {
			final double maxRPM = m_transmission.get().maxRPM,
					speedLeft = leftPower * maxRPM,
					speedRight = rightPower * maxRPM;

			leftMotor.getPIDController().setReference(
					speedLeft, ControlType.kVelocity);
			rightMotor.getPIDController().setReference(
					-speedRight, ControlType.kVelocity);
		} else {
			leftMotor.set(leftPower);
			rightMotor.set(-rightPower);
		}
	}

	private static final double
			LIMIT = 1.0,
			QUICKSTOP_THRESHOLD = 0.2,
			kTurnSens = 0.8, // used to be 1.0
			alpha = 0.1;

	private static double limit(final double v) {
		return Math.abs(v) < LIMIT ? v : LIMIT * Math.signum(v);
	}

	public void curvatureDrive(
			final double linearPower,
			double zRotation,
			final boolean isQuickTurn
	) {
		double overPower, angularPower;

		if (isQuickTurn) {
			if (Math.abs(linearPower) < QUICKSTOP_THRESHOLD) {
				m_accumulator = (1 - alpha) * m_accumulator + alpha * limit(zRotation) * 2;
			}

			overPower = 1.0;
			angularPower = -zRotation;
		} else {
			overPower = 0.0;
			zRotation *= -signum(linearPower);
			angularPower = abs(linearPower) * zRotation * kTurnSens - m_accumulator;

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

	public Velocity getSpeeds() {
		return new Velocity(
				rpm2InPerS(getMotor(Side.LEFT).getEncoder().getVelocity()),
				rpm2InPerS(getMotor(Side.RIGHT).getEncoder().getVelocity()));
	}

	public void stopMotors() {
		m_motors.forEach(CANSparkMax::stopMotor);
	}

	public List<CANSparkMax> getMotors() {
		return m_motors;
	}

	public CANSparkMax getMotorLeft() {
		return m_motors.get(0);
	}

	public CANSparkMax getMotor(final Side side) {
		return m_motors.get(side.idx);
	}

	private double rpm2InPerS(final double rpm) {
		final double rps = rpm / 60.0;
		return rps * m_localizations.rotationsToInches(m_transmission);
	}

	public double getInchesPerSecond(final Side side) {
		return rpm2InPerS(getMotor(side).getEncoder().getVelocity())
			   * side.encoderMultiplier;
	}

	public double getInchesPosition(final Side side) {
		return m_localizations.rotationsToInches(m_transmission)
			   * getMotor(side).getEncoder().getPosition()
			   * side.encoderMultiplier;
	}

	public Kinematics getKinematics() {
		return m_kinematics;
	}

	public Localizations getLocals() {
		return m_localizations;
	}

	public RobotState getState() {
		return m_state;
	}

	public DualTransmission getTransmission() {
		return m_transmission;
	}

	public MyAHRS getAHRS() {
		return m_ahrs;
	}

	public Command getDriveCommand() {
		return Objects.requireNonNullElse(m_driveCommand, Commands.nullCommand());
	}

	public RobotStateEstimator getStateEstimator() {
		return m_robotStateEstimator;
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

	public static class Localizations {
		public double width, length, wheelDiameter, maxVelocity,
				maxAcceleration, maxJerk;

		private Trajectory.Config m_config;

		public PIDF pidDrive;

		private PIDFVA pidTrajectory;

		public PIDFVA getPIDFVA() {
			if (pidTrajectory.kV == -1.0) {
				pidTrajectory.kV = 1 / maxVelocity;
			}

			return pidTrajectory;
		}

		public double wheelCircumference() {
			return Math.PI * wheelDiameter;
		}

		public double wheelRadius() {
			return wheelDiameter / 2.0;
		}

		public double rotationsToInches(final DualTransmission transmission) {
			return wheelCircumference() / transmission.get().ratio;
		}

		public Trajectory.Config getTrajectoryConfig() {
			if (Objects.isNull(m_config)) {
				m_config = new Trajectory.Config(
						Trajectory.FitMethod.HERMITE_CUBIC,
						Trajectory.Config.SAMPLES_FAST,
						dt,
						100,
						PathfinderFRC.DEFAULT_ACC * METER_TO_INCHES,
						PathfinderFRC.DEFAULT_JERK * METER_TO_INCHES
				);
			}

			return m_config;
		}
	}

	public class Velocity {
		private final double linear, angular;

		private Velocity(final double l, final double a) {
			linear = l;
			angular = a;
		}

		public double linear() {
			return linear;
		}

		public double angular() {
			return angular;
		}

		public double left() {
			return linear - angular;
		}

		public double right() {
			return linear + angular;
		}
	}

	public Velocity wheelVelocity(final double l, final double r) {
		final double linear = (m_localizations.wheelRadius() * (l + r)) / 2.0,
				angular = m_localizations.wheelRadius() * (r - l) / m_localizations.width;

		return new Velocity(linear, angular);
	}

	public Velocity driveVelocity(final double linear, final double angular) {
		return new Velocity(linear, angular);
	}

	public static class Kinematics {
		private final DifferentialDrive.Localizations m_localizations;
		private final DifferentialDrive m_chassis;

		public Kinematics(final Localizations localizations, final DifferentialDrive chassis) {
			m_localizations = localizations;
			m_chassis = chassis;
		}

		public Twist forwardKinematics(
				final double wheelDeltaLeft,
				final double wheelDeltaRight
		) {
			final double velocityDelta = (wheelDeltaRight - wheelDeltaLeft) / 2.0,
					rotationDelta = velocityDelta * 2 / m_localizations.width;

			return forwardKinematics(wheelDeltaLeft, wheelDeltaRight, rotationDelta);
		}

		public Twist forwardKinematics(
				final double wheelDeltaLeft,
				final double wheelDeltaRight,
				final double rotationDelta
		) {
			final double dx = (wheelDeltaLeft + wheelDeltaRight) / 2.0;
			return new Twist(dx, 0, rotationDelta);
		}

		public Twist forwardKinematics(
				final Rotation previousHeading,
				final double wheelLeftDelta,
				final double wheelRightDelta,
				final Rotation currentHeading
		) {
			final double dx = (wheelLeftDelta + wheelRightDelta) / 2.0,
					dy = 0.0;

			return new Twist(
					dx,
					dy,
					previousHeading.inverse().rotate(currentHeading).getRadians());
		}

		public Pose integrateForwardKinematics(
				final Pose currentPose,
				final Twist forwardKinematics
		) {
			return currentPose.transform(Pose.exp(forwardKinematics));
		}

		public Velocity inverseKinematics(final Twist velocity) {
			if (Math.abs(velocity.dtheta) < Epsilon) {
				return m_chassis.wheelVelocity(velocity.dx, velocity.dy);
			} else {
				final double dv = m_localizations.width * velocity.dtheta / 2.0;
				return m_chassis.wheelVelocity(velocity.dx - dv, velocity.dx + dv);
			}
		}
	}

	@Override
	public void initDefaultCommand() { }

	@Override
	public void initSendable(final SendableBuilder builder) {
		builder.setSmartDashboardType("West Coast Drive");

		final List<Runnable> updaters = IntStream.range(0, 4).<Runnable>mapToObj(i -> {
			final CANSparkMax spark = m_motors.get(i);
			final String name = SPARK_NAMES[i];

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
