package frc.robot.command.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.ElevatorSetpointManager;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.subsystem.ElevatorSubsystem;

public class WaitForElevatorSetpointCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final ElevatorSetpoint waitForSetpoint;

    public WaitForElevatorSetpointCommand(ElevatorSubsystem elevator, ElevatorSetpoint waitForSetpoint) {
        this.elevator = elevator;
        this.waitForSetpoint = waitForSetpoint;
    }

    @Override
    public boolean isFinished() {
        var waitForPos = ElevatorSetpointManager.getInstance().getSetpoint(waitForSetpoint);
        return elevator.atPosition(waitForPos);
    }
}
