package frc.robot.input;

import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorMovementRequest;
import frc.robot.utils.Vec2;
import java.util.Optional;

public interface MoInput {
    public abstract Optional<ElevatorSetpoint> getElevatorSetpoints();

    public abstract Vec2 getMoveRequest();

    public abstract double getTurnRequest();

    public abstract boolean getReZeroGyro();

    public abstract ElevatorMovementRequest getElevatorMovementRequest();

    public abstract boolean getSaveElevatorSetpoint();

    public abstract boolean getReZeroElevator();
}
