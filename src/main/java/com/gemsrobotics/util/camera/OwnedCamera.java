package com.gemsrobotics.util.camera;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;

/**
 * A class which retains a pointer to an instance of {@link UsbCamera}
 * so that it cannot pass out of our ownership.
 * Notably, this makes it easier to swap camera views during runtime.
 */
@SuppressWarnings("FieldCanBeLocal")
public final class OwnedCamera {
	private static int newId;
	static {
		newId = 0;
	}

	private final UsbCamera m_camera;
	private final CvSink m_sink;

	public OwnedCamera(final UsbCamera camera) {
		m_camera = camera;
		m_sink = new CvSink("sink" + newId++);
		m_sink.setSource(m_camera);
		m_sink.setEnabled(true);
	}

	public UsbCamera getCamera() {
		return m_camera;
	}
}
