package com.gemsrobotics;

import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.subsystems.drive.DrivePorts;
import com.gemsrobotics.subsystems.intake.Manipulator;
import com.gemsrobotics.subsystems.intake.ManipulatorConfig;
import com.gemsrobotics.subsystems.lift.Lift;
import com.gemsrobotics.subsystems.lift.LiftConfig;
import com.gemsrobotics.subsystems.lift.SparkLift;
import com.gemsrobotics.util.PIDF;
import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SerialPort;
import com.moandjiezana.toml.Toml;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import java.util.Objects;
import java.util.Optional;

import static com.gemsrobotics.Config.getConfig;

@SuppressWarnings({"unused", "WeakerAccess"})
public class Hardware {
	private final DifferentialDrive m_chassis;
	private final Optional<SerialPort> m_jevoisPort;
	private final Lift m_lift;
	private final Compressor m_compressor;
	private final Manipulator m_manipulator;

	private static Hardware INSTANCE;

	public static Hardware getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Hardware();
		}

		return INSTANCE;
	}

	protected Hardware() {
		final Toml
				driveCfg = getConfig("drive_config"),
				shifterCfg = getConfig("shifter_config"),
				liftCfg = getConfig("lift_config"),
				manipulatorCfg = getConfig("manipulator_config");

		m_compressor = new Compressor();
		m_lift = new SparkLift(liftCfg.to(LiftConfig.class));
		m_jevoisPort = getJevois();
		m_manipulator = new Manipulator(manipulatorCfg.to(ManipulatorConfig.class));
		m_chassis = new DifferentialDrive(
				driveCfg.getTable("localizations").to(DifferentialDrive.Localizations.class),
				new DoubleSolenoid(0, 1),
				driveCfg.getTable("pidf").to(PIDF.class),
				driveCfg.getTable("ports").to(DrivePorts.class),
				false
		);
	}

	private Optional<SerialPort> getJevois() {
		SerialPort ret = null;

		try {
			ret = new SerialPort(921600, SerialPort.Port.kUSB);
		} catch (final UncleanStatusException use) {
			System.out.println("JeVois not obtainable!");
		}

		return Optional.ofNullable(ret);
	}

	public DifferentialDrive getChassis() {
		return m_chassis;
	}

	public Lift getLift() {
		return m_lift;
	}

	public Compressor getCompressor() {
		return m_compressor;
	}

	public Optional<SerialPort> getJevoisPort() {
		return m_jevoisPort;
	}

	public Manipulator getManipulator() {
		return m_manipulator;
	}
}
