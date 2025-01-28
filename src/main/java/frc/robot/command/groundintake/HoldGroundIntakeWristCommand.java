package frc.robot.command.groundintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.GroundIntakeWristSubsystem;

public class HoldGroundIntakeWristCommand extends Command {
    public enum Direction {
        IN,
        OUT
    };

    private final GroundIntakeWristSubsystem intakeSubsystem;
    private final Direction direction;

    public HoldGroundIntakeWristCommand(GroundIntakeWristSubsystem intakeSubsystem, Direction direction) {
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
