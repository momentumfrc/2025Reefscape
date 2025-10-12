package frc.robot.command.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Prefs;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorMovementRequest;

public class ZeroElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;

    private final Timer currentSenseTimer = new Timer();

    public ZeroElevatorCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        currentSenseTimer.restart();
        elevator.disableReverseLimit();
    }

    @Override
    public void execute() {
        if (elevator.hasZero()) {
            elevator.adjustDirectPower(new ElevatorMovementRequest(0, 0));
            return;
        }

        elevator.moveForElevatorZeroing();

        if (elevator.getElevatorCurrent().gte(Prefs.elevatorZeroCurrentThresh.get())) {
            if (currentSenseTimer.hasElapsed(Prefs.elevatorZeroCurrentTime.get().in(Units.Seconds))) {
                elevator.zeroElevator();
            }
        } else {
            currentSenseTimer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        return elevator.hasZero();
    }
}
