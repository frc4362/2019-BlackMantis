package com.gemsrobotics.commands;

import com.gemsrobotics.util.camera.Limelight;
import edu.wpi.first.wpilibj.command.Command;

import com.gemsrobotics.subsystems.manipulator.Manipulator;

public class AutoPickupCommand extends Command {
    private final Manipulator m_manipulator;
    private final Limelight m_limelight;

    private State m_state;
    private boolean m_isFinished;

    public AutoPickupCommand(
            final Manipulator manipulator,
            final Limelight limelight
    ) {
        m_manipulator = manipulator;
        m_limelight = limelight;
    }

    private enum State {
        APPROACHING, READY_TO_EXTEND, READY_FOR_PICKUP, READY_FOR_DEPARTURE
    }

    @Override
    public void initialize() {
        m_isFinished = false;
        m_state = State.APPROACHING;
    }

    @Override
    public void execute() {
        final var area = m_limelight.getArea();

        if (area > 1.5 && m_state == State.APPROACHING) {
            m_state = State.READY_TO_EXTEND;
            m_manipulator.getHand().set(true);
        }

        if (area > 17 && m_state == State.READY_TO_EXTEND) {
            m_state = State.READY_FOR_PICKUP;
            m_manipulator.getArm().set(true);
        }

        if (area > 29 && m_state == State.READY_FOR_PICKUP) {
            m_state = State.READY_FOR_DEPARTURE;
            m_manipulator.getHand().set(false);
        }

        // if the driver backs out
        if (area < 13 && m_state == State.READY_FOR_PICKUP) {
            m_state = State.APPROACHING;
            m_manipulator.getArm().set(false);
            m_manipulator.getHand().set(false);
        }

        if (area < 20 && m_state == State.READY_FOR_DEPARTURE) {
            m_isFinished = true;
            m_manipulator.getArm().set(false);
        }
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
