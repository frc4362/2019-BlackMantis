package com.gemsrobotics.util.camera;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

@SuppressWarnings("unused")
public class Limelight {
	private final NetworkTableEntry
			m_presentEntry, m_offsetHorizontalEntry, m_offsetVerticalEntry,
			m_areaEntry, m_modeLedEntry, m_modeCameraEntry, m_pipelineEntry;

	public Limelight() {
		final var table = NetworkTableInstance.getDefault().getTable("limelight");
		m_presentEntry = table.getEntry("tv");
		m_offsetHorizontalEntry = table.getEntry("tx");
		m_offsetVerticalEntry = table.getEntry("ty");
		m_areaEntry = table.getEntry("ta");
		m_modeLedEntry = table.getEntry("ledMode");
		m_modeCameraEntry = table.getEntry("cameraMode");
		m_pipelineEntry = table.getEntry("pipeline");
	}

	public enum LEDMode {
		DEFER(0),
		OFF(1),
		BLINK(2),
		ON(3);

		public final int value;

		LEDMode(final int v) {
			value = v;
		}
	}

	public enum CameraMode {
		CV(0),
		DRIVER(1);

		public final int value;

		CameraMode(final int v) {
			value = v;
		}
	}

	public boolean isTargetPresent() {
		return m_presentEntry.getDouble(0) == 1.0;
	}

	public double getOffsetHorizontal() {
		return Math.toRadians(m_offsetHorizontalEntry.getDouble(999));
	}

	public double getOffsetVertical() {
		return Math.toRadians(m_offsetVerticalEntry.getDouble(999));
	}

	public double getArea() {
		return m_areaEntry.getDouble(0.0);
	}

	public void setLEDMode(final LEDMode mode) {
		m_modeLedEntry.setDouble(mode.value);
	}

	public void setCameraMode(final CameraMode mode) {
		m_modeCameraEntry.setDouble(mode.value);
	}

	public void setPipeline(final int p) {
		if (p < 9 && p > 0) {
			m_pipelineEntry.setDouble(p);
		}
	}
}
