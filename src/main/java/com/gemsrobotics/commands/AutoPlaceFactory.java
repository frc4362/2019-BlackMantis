package com.gemsrobotics.commands;

import com.gemsrobotics.commands.any.Wait;
import com.gemsrobotics.subsystems.lift.Lift;
import com.gemsrobotics.subsystems.manipulator.Manipulator;
import edu.wpi.first.wpilibj.command.Command;

import java.util.HashMap;
import java.util.Map;

import static com.gemsrobotics.util.command.Commands.autonOf;
import static com.gemsrobotics.util.command.Commands.commandOf;

public class AutoPlaceFactory {
	private static final long DEBOUNCE_MS = 1000;

	private final Lift m_lift;
	private final Manipulator m_manipulator;
	private final Map<Lift.Position, Long> m_lastCreationTimes;
	private final Map<Lift.Position, Boolean> m_positionReady;

	public AutoPlaceFactory(
			final Lift lift,
			final Manipulator manipulator
	) {
		m_lift = lift;
		m_manipulator = manipulator;
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

			ret = autonOf(
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
