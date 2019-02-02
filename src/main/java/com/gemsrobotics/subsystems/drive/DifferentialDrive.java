package com.gemsrobotics.subsystems.drive;

import com.gemsrobotics.util.DualTransmission;
import com.gemsrobotics.util.PIDF;
import com.gemsrobotics.util.motion.Epsilon;
import com.gemsrobotics.util.motion.Pose;
import com.gemsrobotics.util.motion.Rotation;
import com.gemsrobotics.util.motion.Twist;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class DifferentialDrive extends Subsystem {
	public static final double dt = 0.02;
	private static final double METER_TO_INCHES = 0.025399986284007407;

	private final Kinematics m_kinematics;
	private final Localizations m_localizations;
	private final DualTransmission m_transmission;
	private final RobotState m_state;
	private final List<CANSparkMax> m_motors;

	private boolean m_useVelocityControl;
	private double m_stopAccumulator;
	private String m_name;

	private static final String[] SPARK_NAMES = {
		"Left Front", "Left Back", "Right Front", "Right Back"
	};

	public DifferentialDrive(
			final Localizations localizations,
			final DoubleSolenoid shifter,
			final PIDF pidVars,
			final DrivePorts drivePorts,
			final boolean useVelocityControl
	) {
		final Integer[] ports = drivePorts.get();
		m_localizations = localizations;
		m_transmission = new DualTransmission(shifter);
		m_kinematics = new Kinematics(m_localizations);
		m_state = new RobotState(this);

		m_useVelocityControl = useVelocityControl;
		m_stopAccumulator = 0.0;

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
			pidVars.configure(controller);
			controller.setOutputRange(-1.0, +1.0);
		});
	}

	private static final double
			LIMIT = 1.0,
			QUICKSTOP_THRESHOLD = 0.2,
			kTurnSensitivity = 1.0,
			alpha = 0.1;

	private static double limit(final double v) {
		return Math.abs(v) < LIMIT ? v : LIMIT * Math.signum(v);
	}

	public void drive(final double leftPower, final double rightPower) {
		if (m_useVelocityControl) {
			final double maxRPM = m_transmission.get().maxRPM,
					speedLeft = leftPower * maxRPM,
					speedRight = rightPower * maxRPM;

			getMotor(Side.LEFT).getPIDController()
					.setReference(speedLeft, ControlType.kVelocity);
			getMotor(Side.RIGHT).getPIDController()
					.setReference(-speedRight, ControlType.kVelocity);
		} else {
			getMotor(Side.LEFT).set(leftPower);
			getMotor(Side.RIGHT).set(-rightPower);
		}
	}

	public void curvatureDrive(
			final double linearPower,
			double zRotation,
			final boolean isQuickTurn
	) {
		double overPower, angularPower;

		if (isQuickTurn) {
			if (Math.abs(linearPower) < QUICKSTOP_THRESHOLD) {
				m_stopAccumulator = (1 - alpha) * m_stopAccumulator + alpha * limit(zRotation) * 2;
			}

			overPower = 1.0;
			angularPower = -zRotation;
		} else {
			overPower = 0.0;
			zRotation *= -Math.signum(linearPower);
			angularPower = Math.abs(linearPower) * zRotation * kTurnSensitivity - m_stopAccumulator;

			if (m_stopAccumulator > 1) {
				m_stopAccumulator -= 1;
			} else if (m_stopAccumulator < -1) {
				m_stopAccumulator += 1;
			} else {
				m_stopAccumulator = 0.0;
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

	public DriveSignal getSpeeds() {
		return new DriveSignal(
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
		return getMotor(side).getEncoder().getPosition()
			   * m_localizations.wheelCircumference()
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
		double width, length, wheelDiameter, maxVelocity, maxAcceleration, maxJerk;

		public double wheelCircumference() {
			return Math.PI * wheelDiameter;
		}

		public double rotationsToInches(final DualTransmission transmission) {
			return wheelCircumference() / transmission.get().ratio;
		}

		// TODO make it actually use the config vals
		public Trajectory.Config getTrajectoryConfig() {
			return new Trajectory.Config(
					Trajectory.FitMethod.HERMITE_CUBIC,
					Trajectory.Config.SAMPLES_FAST,
					0.02,
					0.0,
					PathfinderFRC.DEFAULT_ACC * METER_TO_INCHES,
					PathfinderFRC.DEFAULT_JERK * METER_TO_INCHES
			);
		}
	}

	public static class DriveSignal {
		InPerSecond left, right;

		DriveSignal(final double l, final double r) {
			left = new InPerSecond(l);
			right = new InPerSecond(r);
		}
	}

	public static class Kinematics {
		private final DifferentialDrive.Localizations m_localizations;

		public Kinematics(final Localizations localizations) {
			m_localizations = localizations;
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

		public DriveSignal inverseKinematics(final Twist velocity) {
			if (Math.abs(velocity.dtheta) < Epsilon.VALUE) {
				return new DriveSignal(velocity.dx, velocity.dy);
			} else {
				final double dv = m_localizations.width * velocity.dtheta / 2.0;
				return new DriveSignal(velocity.dx - dv, velocity.dx + dv);
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
					velocityEntry = builder.getEntry(name + " Velocity (In/s)"),
					positionEntry = builder.getEntry(name + " Position (In)");

			builder.getEntry(name + " CAN ID").setString(Integer.toString(spark.getDeviceId()));

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
