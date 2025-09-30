package frc.robot.command.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Prefs;
import frc.robot.command.intake.IntakeCommands.Direction;
import frc.robot.subsystem.IntakeWristSubsystem;

public class MoveIntakeWristCommand extends Command {
    private final IntakeWristSubsystem intake;
    private final Direction direction;

    private final Timer wristCurrentTimer = new Timer();

    public MoveIntakeWristCommand(IntakeWristSubsystem intake, Direction direction) {
        this.intake = intake;
        this.direction = direction;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        wristCurrentTimer.restart();
    }

    @Override
    public void execute() {
        if (direction == Direction.IN) {
            intake.wristIn();
        } else {
            intake.wristOut();
        }
    }

    @Override
    public boolean isFinished() {
        return wristCurrentTimer.hasElapsed(Prefs.intakeWristTime.get().in(Units.Seconds));
    }
}
