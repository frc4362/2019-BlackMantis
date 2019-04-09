package com.gemsrobotics.commands;

import com.gemsrobotics.commands.any.Wait;
import com.gemsrobotics.subsystems.lift.Lift;
import com.gemsrobotics.subsystems.manipulator.Manipulator;
import com.gemsrobotics.util.camera.Limelight;
import com.gemsrobotics.util.command.Commands;
import edu.wpi.first.wpilibj.command.Command;

import java.util.HashMap;
import java.util.Map;

import static com.gemsrobotics.util.command.Commands.commandGroupOf;
import static com.gemsrobotics.util.command.Commands.commandOf;

public class AutoPlaceFactory {
	private static final long DEBOUNCE_MS = 1000;
	private static final int ACTUATE_THRESHOLD = 30;

	private final Lift m_lift;
	private final Manipulator m_manipulator;
	private final Map<Lift.Position, Long> m_lastCreationTimes;
	private final Map<Lift.Position, Boolean> m_positionReady;
	private final Limelight m_limelight;

	public AutoPlaceFactory(
			final Lift lift,
			final Manipulator manipulator,
			final Limelight limelight
	) {
		m_lift = lift;
		m_manipulator = manipulator;
		m_limelight = limelight;
		m_lastCreationTimes = new HashMap<>();
		m_positionReady = new HashMap<>();
	}

	public Command makeAutoPlace(final Lift.Position position, final boolean openAfterFinish) {
		final var currentTime = System.currentTimeMillis();

		final Command ret;

		if ((!m_lastCreationTimes.containsKey(position)
			|| (currentTime - m_lastCreationTimes.get(position) > DEBOUNCE_MS))
			&& m_positionReady.getOrDefault(position, true)
		) {
			m_positionReady.put(position, false);

			ret = commandGroupOf(
					new WaitForAreaCommand(m_limelight, ACTUATE_THRESHOLD),
					new LiftMovement(m_lift, position),
					new Wait(50),
					new PlacementSequence(m_manipulator, openAfterFinish),
					commandOf(() -> m_positionReady.put(position, true)));

			m_lastCreationTimes.put(position, currentTime);
		} else {
			ret = null;
		}

		return ret;
	}
}
