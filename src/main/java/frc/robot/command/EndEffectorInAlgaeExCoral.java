package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ElevatorSubsystem;

public class EndEffectorInAlgaeExCoral extends Command {
    private final ElevatorSubsystem elevator;

    public EndEffectorInAlgaeExCoral(ElevatorSubsystem elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.intakeAlgaeCoralExtake();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.endEffectorStop();
    }
}
