package com.gemsrobotics.util.motion;

import com.gemsrobotics.util.Utils;

@SuppressWarnings({"unused", "WeakerAccess"})
public class Rotation {
	protected double m_cosAngle, m_sinAngle;

	public static Rotation identity() {
		return new Rotation();
	}

	public Rotation(final double x, final double y, boolean normalize) {
		m_cosAngle = x;
		m_sinAngle = y;

		if (normalize) {
			normalize();
		}
	}

	public Rotation(final Rotation other) {
		this(other.m_cosAngle, other.m_sinAngle, false);
	}

	public Rotation(final Translation direction, final boolean normalize) {
		this(direction.m_x, direction.m_y, normalize);
	}

	public Rotation() {
		this(1, 0, false);
	}

	public static Rotation fromRadians(final double radians) {
		return new Rotation(Math.cos(radians), Math.sin(radians), false);
	}

	public static Rotation fromDegrees(final double degrees) {
		return fromRadians(Math.toRadians(degrees));
	}

	public void normalize() {
		final double magnitude = Math.hypot(m_cosAngle, m_sinAngle);

		if (magnitude > Epsilon.VALUE) {
			m_sinAngle /= magnitude;
			m_cosAngle /= magnitude;
		} else {
			m_sinAngle = 0;
			m_cosAngle = 1;
		}
	}

	public double cos() {
		return m_cosAngle;
	}

	public double sin() {
		return m_sinAngle;
	}

	public double tan() {
		if (Math.abs(m_cosAngle) < Epsilon.VALUE) {
			if (m_sinAngle >= 0.0) {
				return Double.POSITIVE_INFINITY;
			} else {
				return Double.NEGATIVE_INFINITY;
			}
		} else {
			return m_sinAngle / m_cosAngle;
		}
	}

	public double getRadians() {
		return Math.atan2(m_sinAngle, m_cosAngle);
	}

	public double getDegrees() {
		return Math.toDegrees(getRadians());
	}

	public Rotation rotate(final Rotation other) {
		return new Rotation(
				m_cosAngle * other.m_cosAngle - m_sinAngle * other.m_sinAngle,
				m_cosAngle * other.m_sinAngle + other.m_sinAngle * other.m_cosAngle,
				true);
	}

	public Rotation normal() {
		return new Rotation(-m_sinAngle, m_cosAngle, false);
	}

	public Rotation inverse() {
		return new Rotation(m_cosAngle, -m_sinAngle, false);
	}

	public boolean isParallel(final Rotation other) {
		return Epsilon.epsilonEquals(Translation.cross(toTranslation(), other.toTranslation()), 0.0);
	}

	public Translation toTranslation() {
		return new Translation(m_cosAngle, m_sinAngle);
	}

	@Override
	public String toString() {
		return "Rotation[theta: " + Utils.getDCF().format(getRadians()) + "]";
	}
}
