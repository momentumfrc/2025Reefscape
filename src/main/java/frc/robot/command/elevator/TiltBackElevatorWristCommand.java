package frc.robot.command.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.ElevatorSubsystem;

public class TiltBackElevatorWristCommand extends Command {
    private final ElevatorSubsystem elevator;

    private final Timer wristCurrentTimer = new Timer();

    public TiltBackElevatorWristCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        wristCurrentTimer.restart();
    }

    @Override
    public void execute() {
        elevator.tiltBack();
    }

    @Override
    public boolean isFinished() {
        Current currAmps = elevator.getWristCurrent();
        if (currAmps.gte(MoPrefs.elevatorWristCurrentThreshold.get())) {
            if (wristCurrentTimer.hasElapsed(
                    MoPrefs.elevatorWristCurrentTime.get().in(Units.Seconds))) {
                return true;
            }
        } else {
            wristCurrentTimer.restart();
        }

        return false;
    }
}
