package frc.robot.command.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.IntakeWristSubsystem;

public class HoldIntakeWristCommand extends Command {
    public enum Direction {
        IN,
        OUT
    };

    private final IntakeWristSubsystem intakeSubsystem;
    private final Direction direction;

    public HoldIntakeWristCommand(IntakeWristSubsystem intakeSubsystem, Direction direction) {
        this.intakeSubsystem = intakeSubsystem;
        this.direction = direction;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (direction == Direction.OUT) {
            intakeSubsystem.holdWristOut();
        } else {
            intakeSubsystem.holdWristIn();
        }
    }
}
