package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.prefs.MoPrefs;

public class LEDsSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private final int ledCount = 120; // Might want to fix this.

    private final LEDPattern rainbow = LEDPattern.rainbow(255, 255);
    private static final Distance kLedSpacing = Meters.of(1 / 60.0);
    private final LEDPattern rainbowPattern = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    private LEDPattern dimmed;

    private LEDMode currentMode;

    public enum LEDMode {
        RAINBOW,
        END_EFFECTOR,
        ELEVATOR,
        GROUND_INTAKE,
        CLIMBER
    }

    public LEDsSubsystem() {
        MoPrefs.ledBrightness.subscribe(
                brightness -> {
                    dimmed = rainbowPattern.atBrightness((Dimensionless) brightness);
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
        switch (currentMode) {
            case RAINBOW:
                updateRainbowPattern();
                break;
            case END_EFFECTOR:
                endEffectorColor();
                break;
            case ELEVATOR:
                elevatorColor();
                break;
            case GROUND_INTAKE:
                groundIntakeColor();
                break;
            case CLIMBER:
                climbColor();
                break;
        }
        led.setData(ledBuffer);
    }

    public void updateRainbowPattern() {
        dimmed.applyTo(ledBuffer);
    }

    public void endEffectorColor() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 159, 1, 255);
        }
    }

    public void elevatorColor() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 5, 205, 253);
        }
    }

    public void groundIntakeColor() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
        }
    }

    public void climbColor() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 0, 0);
        }
    }

}
