package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.prefs.MoPrefs;

public class LEDsSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public static final int ledCount = 120; // Might want to fix this.
    public static final Distance kLedSpacing = Meters.of(1 / 60.0);

    private final LEDWriter dimmer;

    public LEDsSubsystem() {
        this.led = new AddressableLED(Constants.leds.port());
        this.ledBuffer = new AddressableLEDBuffer(ledCount);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        dimmer = (i, r, g, b) -> {
            // Clamp RGB values to keep them in the range [0, 255].
            // Otherwise, the casts to byte would result in values like 256 wrapping to 0
            double multiplier = MoPrefs.ledBrightness.get().in(Units.Value);
            ledBuffer.setRGB(
                    i, (int) MathUtil.clamp(r * multiplier, 0, 255), (int) MathUtil.clamp(g * multiplier, 0, 255), (int)
                            MathUtil.clamp(b * multiplier, 0, 255));
        };
    }

    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(ledBuffer, dimmer);
    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }
}
