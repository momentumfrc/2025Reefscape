package frc.robot.command.elevator;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.component.ElevatorSetpointManager;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorPosition;
//import frc.robot.subsystem.ElevatorSubsystem.WristState;

public class MoveElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final ElevatorPosition position;

    //private ElevatorSetpoint currentElevatorSetpoint;
    //private WristState state;

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

    /*private void adjustSmartPosition(ElevatorSubsystem elevator, ElevatorPosition position) {
        switch(state) {
            case NORMAL:
            default:
                if (ElevatorSetpointManager.getInstance().getSetpoint(setpoint))
                if (!elevator.atHeight(position) && !elevator.stowedWrist(position)) elevator.stowWrist();
                else if (elevator.stowedWrist(position)) elevator.adjustElevatorSmartPosition(position);
                else elevator.adjustWristSmartPosition(position);
                break;
            case HOLDING:
                break;
            case RETURNING:
                break;
        }
    }*/

    @Override
    public void execute() {
        elevator.adjustElevatorSmartPosition(position);
        elevator.adjustWristSmartPosition(position);
    }
}
