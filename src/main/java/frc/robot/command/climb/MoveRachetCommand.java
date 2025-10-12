package frc.robot.command.climb;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Prefs;
import frc.robot.subsystem.ClimberSubsystem;
import frc.robot.subsystem.ClimberSubsystem.RachetState;

public class MoveRachetCommand extends Command {
    private final ClimberSubsystem climber;
    private final RachetState desiredState;

    private final Timer timer;
    private boolean skipTimer = false;

    public MoveRachetCommand(ClimberSubsystem climber, RachetState desiredState) {
        this.climber = climber;
        this.desiredState = desiredState;
        this.timer = new Timer();

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        RachetState currState = climber.getRachetState();
        skipTimer = currState != null && currState == desiredState;
        timer.restart();
    }

    @Override
    public void execute() {
        climber.moveRachet(desiredState);
        climber.idleClimber();
    }

    @Override
    public boolean isFinished() {
        return skipTimer
                || timer.hasElapsed(Prefs.climberRachetLockoutTime.get().in(Units.Seconds));
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            climber.finishMoveRachet(desiredState);
        }
        timer.stop();
    }
}
