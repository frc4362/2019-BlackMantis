package com.gemsrobotics.commands;

import com.gemsrobotics.Hardware;
import com.gemsrobotics.subsystems.inventory.Inventory;
import com.gemsrobotics.subsystems.inventory.ManualInventory;
import com.gemsrobotics.util.camera.Limelight;
import com.gemsrobotics.util.joy.Gemstick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

public class LEDListener extends Command {
    private final Relay m_light;
    private final XboxController m_controller;
    private final Limelight m_limelight;
    private final Inventory m_inventory;

    public LEDListener(
            final Relay light,
            final XboxController controller,
            final Limelight limelight,
            final Inventory inventory
    ) {
        m_light = light;
        m_controller = controller;
        m_limelight = limelight;
        m_inventory = inventory;
    }

    @Override
    public void execute() {
        if (m_inventory instanceof ManualInventory) {
            m_light.set(m_controller.getRawButton(3) ? Relay.Value.kOn : Relay.Value.kOff);
        } else {
            final var isAttemptingPickup = m_controller.getPOV() == Gemstick.POVState.W.getValue();

            if (m_controller.getPOV() == -1) {
                m_light.set(Relay.Value.kOff);
            } else if (isAttemptingPickup && m_limelight.isTargetPresent()) {
                m_light.set(Relay.Value.kOn);
            } else if (isAttemptingPickup) {
                m_light.set((System.currentTimeMillis() / 300L) % 2 == 0 ? Relay.Value.kOn : Relay.Value.kOff);
            }
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
