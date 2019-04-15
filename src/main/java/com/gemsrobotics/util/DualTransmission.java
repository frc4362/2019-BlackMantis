package com.gemsrobotics.util;

import com.gemsrobotics.commands.ShiftScheduler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.OpenOption;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

public class DualTransmission {
	public enum Gear {
		LOW(6.73, 22.08 * 12), HIGH(13.85, 10.73 * 12);

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
		//TODO
//		final var path = Paths.get("/home/lvuser/shifting.log");
//
//		try {
//			if (Files.isRegularFile(path)) {
//				if (!Files.exists(path)) {
//					Files.createFile(path);
//				}
//			}
//
//			Files.write(
//					path,
//					String.format("[%f] Shifted to %s",
//							Timer.getFPGATimestamp(),
//							gear.toString()).getBytes(),
//					StandardOpenOption.APPEND);
//		} catch (final IOException ioex) {
//			System.out.println("Failed log");
//		}
		System.out.printf("Shifted to %s\n", gear.toString());

		m_shifter.set(gear == Gear.HIGH);
	}
}
