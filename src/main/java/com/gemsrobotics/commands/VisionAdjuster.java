package com.gemsrobotics.commands;

import com.gemsrobotics.subsystems.adjuster.LateralAdjuster;
import com.gemsrobotics.subsystems.inventory.Inventory;
import com.gemsrobotics.subsystems.lift.Lift;
import com.gemsrobotics.util.camera.Limelight;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

import static java.lang.Math.abs;

public class VisionAdjuster extends Command {
	private static final double OVERRIDE_THRESHOLD = 0.3;

	private final LateralAdjuster m_adjuster;
	private final Limelight m_limelight;
	private final Inventory m_inventory;
	private final XboxController m_controller;
	private final Lift m_lift;

	private boolean m_isOverridingLast;

	public VisionAdjuster(
			final LateralAdjuster lateralAdjuster,
			final Limelight limelight,
			final Inventory inventory,
			final XboxController controller,
			final Lift lift
	) {
		m_adjuster = lateralAdjuster;
		m_limelight = limelight;
		m_inventory = inventory;
		m_controller = controller;
		m_lift = lift;
	}

	@Override
	public void initialize() {
		m_isOverridingLast = false;
	}

	@Override
	public void execute() {
		final double driveVal = deadband(m_controller.getX(GenericHID.Hand.kRight));

		boolean isOverriding = m_isOverridingLast || driveVal != 0.0;

		if (isOverriding) {
			m_adjuster.drive(driveVal * -m_adjuster.kLatVolts);
		} else {
			final var hasCargo = m_inventory.getCurrentPiece() == Inventory.GamePiece.CARGO;

			if (m_controller.getRawButton(1)) {
				m_adjuster.setPercent(0.5);
			} else if (!m_lift.isBlockingCamera() && m_limelight.isTargetPresent() && !hasCargo) {
				m_adjuster.alignRadians(-m_limelight.getOffsetHorizontal());
			}
		}

		if (m_isOverridingLast && m_controller.getRawButton(9)) {
			isOverriding = false;
		}

		m_isOverridingLast = isOverriding;
	}

	private static double deadband(final double val) {
		return abs(val) > OVERRIDE_THRESHOLD ? val : 0.0;
	}

	@Override
	public boolean isFinished() {
		return m_adjuster.isDisabled();
	}
}
