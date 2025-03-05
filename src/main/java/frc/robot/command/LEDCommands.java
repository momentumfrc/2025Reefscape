package frc.robot.command;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystem.LEDsSubsystem;
import frc.robot.utils.LEDUtils;

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

    public static Command climberSlowedPattern(LEDsSubsystem leds) {
        Color[] tails = LEDUtils.getColorTails(
                new Color[] {LEDUtils.MOMENTUM_BLUE, LEDUtils.MOMENTUM_PURPLE}, Color.kBlack, 12, 20);
        LinearVelocity scrollSpd = Units.MetersPerSecond.of(1.2);
        int split = LEDsSubsystem.ledCount / 2;
        var firstHalf = LEDUtils.scrollBuffer(tails, scrollSpd, LEDsSubsystem.kLedSpacing)
                .reversed();
        var secondHalf = LEDUtils.scrollBuffer(tails, scrollSpd, LEDsSubsystem.kLedSpacing);

        return ledPatternCommand(LEDUtils.stack(firstHalf, LEDUtils.clip(secondHalf, split, -1)), leds);
    }

    public static Command wristInDangerPattern(LEDsSubsystem leds) {
        return ledPatternCommand(LEDPattern.solid(Color.kRed).blink(Units.Seconds.of(0.25)), leds);
    }

    private LEDCommands() {
        throw new UnsupportedOperationException("LEDCommands is a static class");
    }
}
