package com.gemsrobotics.commands;

import com.ctre.phoenix.CANifier;
import com.gemsrobotics.Hardware;
import com.gemsrobotics.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.subsystems.inventory.Inventory;
import com.gemsrobotics.util.DualTransmission;
import com.gemsrobotics.util.camera.Limelight;
import com.gemsrobotics.util.joy.Gemstick;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import java.awt.*;

import static java.lang.Math.floor;
import static java.lang.Math.sin;

public class LEDController {
    private final Inventory m_inventory;
    private final CANifier m_canifier;
    private final Limelight m_limelight;
    private final XboxController m_controller;
    private final DifferentialDrive m_chassis;

    private Color m_lastColor;
    private double m_lastIntensity;

    public LEDController(final XboxController controller) {
        m_inventory = Hardware.getInstance().getInventory();
        m_canifier = Hardware.getInstance().getCANifier();
        m_limelight = Hardware.getInstance().getLimelight();
        m_chassis = Hardware.getInstance().getChassis();
        m_controller = controller;

        m_lastColor = Color.BLACK;
        m_lastIntensity = 0.0;
    }

    public void writePeriodicOutputs() {
        final var isAttemptingPickup = m_controller.getPOV() == Gemstick.POVState.W.getValue();
        final var isHighGear = m_chassis.getTransmission().get() == DualTransmission.Gear.HIGH;

        if (DriverStation.getInstance().isDisabled()) { // softly breathe while disabled
            setLEDColor(Color.PINK, 0.3 * calculatePulseIntensity(6));
        } else if (isHighGear) {
            setLEDColor(Color.RED, 0.3);
        } else if (m_inventory.hasCargo()) {
            setLEDColor(Color.RED, 1.0);
        } else if (isAttemptingPickup) { // pulse if attempting pickup but no target, otherwise hold solid
            final var isTargetPresent = m_limelight.isTargetPresent();
            setLEDColor(Color.ORANGE, isTargetPresent ? 1.0 : 0.7 * calculatePulseIntensity(1.25));
        } else if (Hardware.getInstance().getManipulator().getHand().get()) { // flash bright yellow when hand is closed
            setLEDColor(Color.YELLOW, 1.0);
        } else {
            setLEDColor(Color.BLACK, 0.0);
        }
    }

    private double calculatePulseIntensity(final double wavelengthSeconds) {
        return (sin(Timer.getFPGATimestamp() / wavelengthSeconds) + 1) / 2;
    }

    private void setLEDColor(final Color color, final double intensity) {
        if (!m_lastColor.equals(color) || m_lastIntensity != intensity) {
            m_lastColor = color;
            m_lastIntensity = intensity;

            m_canifier.setLEDOutput((color.getGreen() / 255.0) * intensity, CANifier.LEDChannel.LEDChannelA);
            m_canifier.setLEDOutput((color.getBlue() / 255.0) * intensity, CANifier.LEDChannel.LEDChannelB);
            m_canifier.setLEDOutput((color.getRed() / 255.0) * intensity, CANifier.LEDChannel.LEDChannelC);
        }
    }
}
