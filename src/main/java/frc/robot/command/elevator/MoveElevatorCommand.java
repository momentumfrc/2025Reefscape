package frc.robot.command.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.ElevatorSetpointManager;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorPosition;

public class MoveElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final ElevatorPosition position;

    public MoveElevatorCommand(ElevatorSubsystem elevator, ElevatorPosition toPosition) {
        this.elevator = elevator;
        this.position = toPosition;

        addRequirements(elevator);
    }

    public static MoveElevatorCommand forSetpoint(ElevatorSubsystem elevator, ElevatorSetpoint setpoint) {
        return new MoveElevatorCommand(
                elevator, ElevatorSetpointManager.getInstance().getSetpoint(setpoint));
    }

    public static Command goToSetpointAndEnd(ElevatorSubsystem elevator, ElevatorSetpoint setpoint) {
        return forSetpoint(elevator, setpoint).raceWith(new WaitForElevatorSetpointCommand(elevator, setpoint));
    }

    @Override
    public void execute() {
        elevator.adjustSmartPosition(position);
    }
}
