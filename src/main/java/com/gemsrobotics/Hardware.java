package com.gemsrobotics;

import com.gemsrobotics.subsystems.inventory.DumbUltrasonicInventory;
import com.gemsrobotics.subsystems.inventory.Inventory;
import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.subsystems.drive.DrivePorts;
import com.gemsrobotics.subsystems.inventory.UltrasonicInventoryConfig;
import com.gemsrobotics.subsystems.manipulator.Manipulator;
import com.gemsrobotics.subsystems.manipulator.ManipulatorConfig;
import com.gemsrobotics.subsystems.lift.LiftConfig;
import com.gemsrobotics.subsystems.lift.Lift;
import com.gemsrobotics.util.MyAHRS;
import com.gemsrobotics.util.camera.Limelight;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import com.moandjiezana.toml.Toml;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import java.util.List;
import java.util.Objects;

import static com.gemsrobotics.Config.getConfig;

@SuppressWarnings({"unused", "WeakerAccess"})
public class Hardware {
	private final DifferentialDrive m_chassis;
	private final Lift m_lift;
	private final Compressor m_compressor;
	private final Manipulator m_manipulator;
	private final Limelight m_limelight;
	private final Inventory m_inventory;
	private final MyAHRS m_ahrs;
	private final Relay m_leds;

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
				manipulatorCfg = getConfig("manipulator_config"),
				inventoryCfg = getConfig("inventory_config"),
				ledsCfg = getConfig("relay_config");

		final List<Long> shifterPorts = shifterCfg.getList("ports");

		final var ultraCfg = inventoryCfg.to(UltrasonicInventoryConfig.class);
		m_inventory = new DumbUltrasonicInventory(ultraCfg);
		m_leds = new Relay(ledsCfg.getLong("port").intValue());
		m_limelight = new Limelight();
		m_compressor = new Compressor();
		m_lift = new Lift(liftCfg.to(LiftConfig.class));
		m_ahrs = new MyAHRS(SPI.Port.kMXP);
		m_manipulator = new Manipulator(manipulatorCfg.to(ManipulatorConfig.class));

		final DoubleSolenoid shifter = new DoubleSolenoid(
				shifterPorts.get(0).intValue(),
				shifterPorts.get(1).intValue()
		);

		m_chassis = new DifferentialDrive(
				driveCfg.getTable("ports").to(DrivePorts.class),
				driveCfg.getTable("localizations").to(DifferentialDrive.Localizations.class),
				shifter,
				m_ahrs,
				false
		);
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

	public Manipulator getManipulator() {
		return m_manipulator;
	}

	public Limelight getLimelight() {
		return m_limelight;
	}

	public Inventory getInventory() {
		return m_inventory;
	}

	public MyAHRS getAHRS() {
		return m_ahrs;
	}

	public Relay getLEDs() {
		return m_leds;
	}
}
