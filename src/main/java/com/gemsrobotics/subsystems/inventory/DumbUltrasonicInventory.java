package com.gemsrobotics.subsystems.inventory;

public class DumbUltrasonicInventory extends UltrasonicInventory {
	public DumbUltrasonicInventory(final UltrasonicInventoryConfig cfg) {
		super(cfg);
	}

	@Override
	public boolean hasPanel() {
		return !super.hasCargo();
	}
}
