package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.LEDsSubsystem;
import frc.robot.subsystem.LEDsSubsystem.LEDMode;

public class LEDsCommand extends Command {
    private final LEDsSubsystem ledSubsystem;
    private final LEDMode mode;

    public LEDsCommand(LEDsSubsystem ledSubsystem, LEDMode mode) {
        this.ledSubsystem = ledSubsystem;
        this.mode = mode;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.setMode(mode);
    }
}
