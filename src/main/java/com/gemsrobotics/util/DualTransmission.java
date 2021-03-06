package com.gemsrobotics.util;

import edu.wpi.first.wpilibj.Solenoid;

public class DualTransmission {
    public enum Gear {
        LOW(6.73, 120), HIGH(13.85, 0);

        public final double ratio, topSpeed;

        Gear(final double r, final double ts) {
            ratio = r;
            topSpeed = ts;
        }
    }

    private final Solenoid m_shifter;

    public DualTransmission(final Solenoid shifter) {
        m_shifter = shifter;
    }

    public Gear get() {
        return m_shifter.get() ? Gear.HIGH : Gear.LOW;
    }

    public void set(final Gear gear) {
        System.out.printf("Shifted to %s\n", gear.toString());
        m_shifter.set(gear == Gear.HIGH);
    }
}
