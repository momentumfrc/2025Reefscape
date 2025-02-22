package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDsSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private final int ledCount = 120; // Might want to fix this.

    private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    private static final Distance kLedSpacing = Meters.of(1 / 120.0);
    private final LEDPattern rainbowPattern = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    public LEDsSubsystem() {
        this.led = new AddressableLED(Constants.leds.port());
        this.ledBuffer = new AddressableLEDBuffer(ledCount);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
        updateRainbowPattern();
    }

    public void updateRainbowPattern() {
        rainbowPattern.applyTo(ledBuffer);
    }
}
