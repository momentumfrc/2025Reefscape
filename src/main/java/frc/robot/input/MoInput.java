package frc.robot.input;

import frc.robot.utils.Vec2;

public interface MoInput {
    public abstract Vec2 getMoveRequest();

    public abstract double getTurnRequest();

    public abstract boolean getReZeroGyro();

    public abstract double getClimberMoveRequest();
}
