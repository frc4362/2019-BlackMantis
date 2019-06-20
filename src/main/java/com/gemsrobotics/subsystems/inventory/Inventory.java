package com.gemsrobotics.subsystems.inventory;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public abstract class Inventory {
	public enum GamePiece {
		PANEL,
		CARGO,
		NONE
	}

	public abstract boolean hasPanel();
	public abstract boolean hasCargo();

	public final boolean isEmpty() {
		return !hasCargo() && !hasPanel();
	}

	public final GamePiece getCurrentPiece() {
		if (hasPanel()) {
			return GamePiece.PANEL;
		} else if (hasCargo()) {
			return GamePiece.CARGO;
		} else {
			return GamePiece.NONE;
		}
	}
}
