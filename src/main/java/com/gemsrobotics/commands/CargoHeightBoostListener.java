package com.gemsrobotics.commands;

import com.gemsrobotics.subsystems.inventory.Inventory;
import com.gemsrobotics.subsystems.lift.Lift;
import com.gemsrobotics.subsystems.manipulator.Manipulator;
import edu.wpi.first.wpilibj.command.Command;

public class CargoHeightBoostListener extends Command {
	private final Lift m_lift;
	private final Manipulator m_intake;
	private final Inventory m_inventory;

	private boolean m_hadCargoLast;

	public CargoHeightBoostListener(
			final Lift lift,
			final Manipulator intake,
			final Inventory inventory
	) {
		m_lift = lift;
		m_intake = intake;
		m_inventory = inventory;
	}

	@Override
	public void initialize() {
		m_hadCargoLast = false;
	}

	@Override
	public void execute() {
		final boolean hasCargo = m_inventory.hasCargo();

		if (!m_hadCargoLast
			&& hasCargo
			&& m_intake.getCurrentRunMode() == Manipulator.RunMode.INTAKING
			&& m_lift.getSetPosition() == Lift.Position.BOTTOM
		) {
			m_lift.setPreset(Lift.Position.CARGO_SHIP);
		}

		m_hadCargoLast = hasCargo;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
