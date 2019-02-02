package com.gemsrobotics.util.command;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Provides delays to run before the auton if delays are enabled in an instance of
 * {@link frc.team4362.util.BetterTimedRobot}, in the form of milliseconds to be passed
 * to an instance of {@link frc.team4362.commands.any.Wait}
 */
public class DelayedAutonFactory {
	// the key in SmartDashboard Preferences
	// which may be used to set the auton delay
	private static final String AUTON_DELAY_AMOUNT_KEY = "DS";

	/**
	 * @return The preferred auton delay in milliseconds.
	 * Intended to be passed to create an instance of {@link frc.team4362.commands.any.Wait}
	 */
	public static long getAppropriateDelay() {
		final int amount = Preferences.getInstance().getInt(AUTON_DELAY_AMOUNT_KEY, 0);
		return amount * 1000;
	}
}
