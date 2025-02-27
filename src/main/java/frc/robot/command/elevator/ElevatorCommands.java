package frc.robot.command.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.component.ElevatorSetpointManager;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.subsystem.ElevatorSubsystem;

public class ElevatorCommands {
    public static Command waitForSetpoint(ElevatorSubsystem elevator, ElevatorSetpoint setpoint) {
        return Commands.waitUntil(() -> elevator.atPosition(
                        ElevatorSetpointManager.getInstance().getSetpoint(setpoint)))
                .withName(String.format("WaitForElevatorSetpoint(%s)", setpoint.name()));
    }

    public static Command holdSetpoint(ElevatorSubsystem elevator, ElevatorSetpoint setpoint) {
        return Commands.runOnce(() -> elevator.reZeroWrist(), elevator)
                .andThen(Commands.run(
                                () -> elevator.adjustPosition(
                                        ElevatorSetpointManager.getInstance().getSetpoint(setpoint)),
                                elevator)
                        .withName(String.format("HoldElevatorSetpoint(%s)", setpoint.name())));
    }

    public static Command moveToSetpoint(ElevatorSubsystem elevator, ElevatorSetpoint setpoint) {
        return Commands.deadline(waitForSetpoint(elevator, setpoint), holdSetpoint(elevator, setpoint))
                .withName(String.format("MoveToElevatorSetpoint(%s)", setpoint.name()));
    }

    private ElevatorCommands() {
        throw new UnsupportedOperationException("ElevatorCommands is a static class");
    }
}
