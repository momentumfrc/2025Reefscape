package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ElevatorSubsystem;

public class EndEffectorExAlgaeInCoral extends Command {
    private final ElevatorSubsystem elevator;

    public EndEffectorExAlgaeInCoral(ElevatorSubsystem elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.extakeAlgaeCoralIntake();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.endEffectorStop();
    }
}
