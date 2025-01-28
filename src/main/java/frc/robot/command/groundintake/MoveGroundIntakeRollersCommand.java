package frc.robot.command.groundintake;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.GroundIntakeRollerSubsystem;

public class MoveGroundIntakeRollersCommand extends Command {
    public enum Direction {
        SHOOT,
        INTAKE
    };

    private final GroundIntakeRollerSubsystem intake;
    private final Direction direction;

    private final Timer rollerCurrentTimer = new Timer();

    public MoveGroundIntakeRollersCommand(GroundIntakeRollerSubsystem intake, Direction direction) {
        this.intake = intake;
        this.direction = direction;

        addRequirements(intake);
    }

    public void initialize() {
        rollerCurrentTimer.start();
    }


    @Override
    public void execute() {
        if (direction == Direction.SHOOT) {
            intake.rollerShoot();
        } else {
            intake.rollerIntake();
        }
    }

    private boolean isBallHeld() {
        // TODO: figure this out later

        //Possible current sensing?
        Current currAmps = intake.getRollerCurrent();
        if (currAmps.gte(MoPrefs.intakeRollerCurrentThreshold.get())) {
            if (rollerCurrentTimer.hasElapsed(
                MoPrefs.intakeRollerCurrentTime.get().in(Units.Seconds))) {
                    return true;
                }
            } else {
            rollerCurrentTimer.restart();
        }

        return false;
    }

    @Override
    public boolean isFinished() {
        return isBallHeld();
    }
}

