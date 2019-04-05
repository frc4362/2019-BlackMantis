package com.gemsrobotics.commands;

import com.gemsrobotics.util.DualTransmission;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Command;

public class ShifterListener extends Command {
    private final Relay m_light;
    private final DualTransmission m_shifter;

    private DualTransmission.Gear m_lastGear;

    public ShifterListener(final Relay light, final DualTransmission shifter) {
        m_light = light;
        m_shifter = shifter;
    }

    @Override
    public void execute() {
        final var gear = m_shifter.get();

        if (gear != m_lastGear) {
            switch (gear) {
                case HIGH:
                    m_light.set(Relay.Value.kOff);
                    break;
                case LOW:
                default:
                    m_light.set(Relay.Value.kOn);
                    break;
            }
        }

        m_lastGear = gear;
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
