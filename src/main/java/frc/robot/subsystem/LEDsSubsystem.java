package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.prefs.MoPrefs;
import java.util.EnumMap;

public class LEDsSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private final int ledCount = 120; // Might want to fix this.

    private final EnumMap<LEDMode, LEDPattern> patterns = new EnumMap<>(LEDMode.class);
    private final EnumMap<LEDMode, LEDPattern> dimmed = new EnumMap<>(LEDMode.class);

    private static final Distance kLedSpacing = Meters.of(1 / 60.0);

    private LEDMode currentMode;

    public enum LEDMode {
        RAINBOW,
        END_EFFECTOR,
        ELEVATOR,
        GROUND_INTAKE,
        CLIMBER
    }

    public LEDsSubsystem() {
        patterns.put(
                LEDMode.RAINBOW,
                LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing));
        patterns.put(LEDMode.END_EFFECTOR, LEDPattern.solid(new Color(159, 1, 255)));
        patterns.put(LEDMode.ELEVATOR, LEDPattern.solid(new Color(5, 205, 253)));
        patterns.put(LEDMode.GROUND_INTAKE, LEDPattern.solid(new Color(0, 255, 0)));
        patterns.put(LEDMode.CLIMBER, LEDPattern.solid(new Color(255, 0, 0)));

        MoPrefs.ledBrightness.subscribe(
                brightness -> {
                    for (LEDMode mode : LEDMode.values()) {
                        dimmed.put(mode, patterns.get(mode).atBrightness((Dimensionless) brightness));
                    }
                },
                true);

        this.led = new AddressableLED(Constants.leds.port());
        this.ledBuffer = new AddressableLEDBuffer(ledCount);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        currentMode = LEDMode.RAINBOW;
    }

    public void setMode(LEDMode mode) {
        this.currentMode = mode;
    }

    @Override
    public void periodic() {
        dimmed.get(currentMode).applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
}
