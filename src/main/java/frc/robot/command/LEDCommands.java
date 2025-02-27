package frc.robot.command;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystem.LEDsSubsystem;

public class LEDCommands {
    private static Command ledPatternCommand(LEDPattern pattern, LEDsSubsystem leds) {
        return Commands.run(() -> leds.applyPattern(pattern), leds).ignoringDisable(true);
    }

    public static Command defaultPattern(LEDsSubsystem leds) {
        return ledPatternCommand(
                LEDPattern.rainbow(255, 255)
                        .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), LEDsSubsystem.kLedSpacing),
                leds);
    }

    public static Command endEffectorPattern(LEDsSubsystem leds) {
        return ledPatternCommand(LEDPattern.solid(new Color(159, 1, 255)), leds);
    }

    public static Command elevatorPattern(LEDsSubsystem leds) {
        return ledPatternCommand(LEDPattern.solid(new Color(5, 205, 253)), leds);
    }

    public static Command groundIntakePattern(LEDsSubsystem leds) {
        return ledPatternCommand(LEDPattern.solid(new Color(0, 255, 0)), leds);
    }

    public static Command climberPattern(LEDsSubsystem leds) {
        return ledPatternCommand(LEDPattern.solid(new Color(255, 0, 0)), leds);
    }

    public static Command wristInDangerPattern(LEDsSubsystem leds) {
        return ledPatternCommand(LEDPattern.solid(Color.kRed).blink(Units.Seconds.of(0.25)), leds);
    }

    private LEDCommands() {
        throw new UnsupportedOperationException("LEDCommands is a static class");
    }
}
