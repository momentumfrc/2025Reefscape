package frc.robot.command.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.IntakeWristSubsystem;

public class MoveIntakeWristCommand extends Command {
    public enum Direction {
        OUT,
        IN
    };

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
        wristCurrentTimer.start();
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
        Current currAmps = intake.getWristCurrent();
        if (currAmps.gte(MoPrefs.intakeWristCurrentThreshold.get())) {
            if (wristCurrentTimer.hasElapsed(
                    MoPrefs.intakeWristCurrentTime.get().in(Units.Seconds))) {
                return true;
            }
        } else {
            wristCurrentTimer.restart();
        }

        return false;
    }
}
