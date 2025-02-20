package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystem.EndEffectorSubsystem;

public class EndEffectorCommands {
    public static Command ExAlgaeInCoral(EndEffectorSubsystem endEffector) {
        return Commands.run(endEffector::extakeAlgaeCoralIntake, endEffector);
    }

    public static Command InAlgaeExCoral(EndEffectorSubsystem endEffector) {
        return Commands.run(endEffector::intakeAlgaeCoralExtake, endEffector);
    }

    public static Command IdleEndEffector(EndEffectorSubsystem endEffector) {
        return Commands.run(endEffector::endEffectorStop, endEffector);
    }
}
