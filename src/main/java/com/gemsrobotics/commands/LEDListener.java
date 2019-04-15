package com.gemsrobotics.commands;

import com.gemsrobotics.util.camera.Limelight;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

public class LEDListener extends Command {
    private final Relay m_light;
    private final XboxController m_controller;
    private final Limelight m_limelight;

    public LEDListener(
            final Relay light,
            final XboxController controller,
            final Limelight limelight
    ) {
        m_light = light;
        m_controller = controller;
        m_limelight = limelight;
    }

    @Override
    public void execute() {
        m_light.set(m_controller.getRawButton(3) ? Relay.Value.kOn : Relay.Value.kOff);

//        final var isAttemptingPickup = m_controller.getPOV() == Gemstick.POVState.W.getValue();
//
//        if (m_controller.getPOV() == -1) {
//            m_light.set(Relay.Value.kOff);
//        } else if (isAttemptingPickup && m_limelight.isTargetPresent()) {
//            m_light.set(Relay.Value.kOn);
//        } else if (isAttemptingPickup) {
//            m_light.set((System.currentTimeMillis() / 300L) % 2 == 0 ? Relay.Value.kOn : Relay.Value.kOff);
//        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
