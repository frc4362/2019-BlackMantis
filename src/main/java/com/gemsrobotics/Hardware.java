package com.gemsrobotics;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.gemsrobotics.subsystems.adjuster.LateralAdjuster;
import com.gemsrobotics.subsystems.adjuster.LateralAdjusterConfig;
import com.gemsrobotics.subsystems.inventory.*;
import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.subsystems.drive.DrivePorts;
import com.gemsrobotics.subsystems.manipulator.Manipulator;
import com.gemsrobotics.subsystems.manipulator.ManipulatorConfig;
import com.gemsrobotics.subsystems.lift.LiftConfig;
import com.gemsrobotics.subsystems.lift.Lift;
import com.gemsrobotics.subsystems.pto.PTO;
import com.gemsrobotics.subsystems.pto.PTOConfig;
import com.gemsrobotics.util.MyAHRS;
import com.gemsrobotics.util.camera.Limelight;
import com.gemsrobotics.util.drivers.LazyTalonSRX;
import edu.wpi.first.wpilibj.*;
import com.moandjiezana.toml.Toml;

import java.util.Objects;

import static com.gemsrobotics.Config.getConfig;

@SuppressWarnings({"unused", "WeakerAccess"})
public class Hardware {
	private final LateralAdjuster m_lateral;
	private final DifferentialDrive m_chassis;
	private final Lift m_lift;
	private final Compressor m_compressor;
	private final Manipulator m_manipulator;
	private final Limelight m_limelight;
	private final Inventory m_inventory;
	private final MyAHRS m_ahrs;
	private final PTO m_pto;
	private final DoubleSolenoid m_legsBack;
	private final Solenoid m_legsFront;
	private final TalonSRX m_rollers;
	private final CANifier m_canifier;

	private static Hardware INSTANCE;

	public static Hardware getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Hardware();
		}

		return INSTANCE;
	}

	protected Hardware() {
		final Toml
				driveCfg = getConfig("drive"),
				shifterCfg = getConfig("shifter"),
				liftCfg = getConfig("lift"),
				manipulatorCfgRaw = getConfig("manipulator"),
				inventoryCfg = getConfig("inventory"),
				lateralCfg = getConfig("lateralAdjuster"),
				ptoCfg = getConfig("pto");

		final var reflectiveCfg = inventoryCfg.to(ReflectiveInventoryConfig.class);
		m_inventory = new ReflectiveInventory(reflectiveCfg.port);
		m_limelight = new Limelight();
		m_compressor = new Compressor();
		m_lift = new Lift(liftCfg.to(LiftConfig.class));
		m_ahrs = new MyAHRS(SPI.Port.kMXP);

		final var manipulatorConfig = manipulatorCfgRaw.to(ManipulatorConfig.class);
		m_manipulator = new Manipulator(manipulatorConfig);
		m_legsFront = new Solenoid(manipulatorConfig.extenderPort);
		m_legsBack = new DoubleSolenoid(6, 7);
		m_pto = new PTO(ptoCfg.to(PTOConfig.class));
		m_rollers = new LazyTalonSRX(7);
		m_lateral = new LateralAdjuster(lateralCfg.to(LateralAdjusterConfig.class));

		final var shifter = new Solenoid(shifterCfg.getLong("port").intValue());
		m_chassis = new DifferentialDrive(
				driveCfg.getTable("ports").to(DrivePorts.class),
				driveCfg.getTable("specifications").to(DifferentialDrive.Specifications.class),
				shifter,
				m_ahrs
		);

		m_canifier = new CANifier(60);
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

	public LateralAdjuster getLateralAdjuster() {
		return m_lateral;
	}

	public PTO getPTO() {
		return m_pto;
	}

	public DoubleSolenoid getBackLegs() {
		return m_legsBack;
	}

	public Solenoid getCargoIntake() {
		return m_legsFront;
	}

	public TalonSRX getRollers() {
		return m_rollers;
	}

	public CANifier getCANifier() {
		return m_canifier;
	}
}
