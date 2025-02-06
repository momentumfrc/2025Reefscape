package frc.robot.command.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystem.IntakeRollerSubsystem;
import frc.robot.subsystem.IntakeWristSubsystem;

public class IntakeCommands {
    public enum Direction {
        IN,
        OUT;
    }

    public static Command holdIntakeRollerCommand(IntakeRollerSubsystem roller) {
        return Commands.run(roller::stopRollerMotor, roller).withName("HoldIntakeRollerCommand");
    }

    public static Command holdIntakeWristCommand(IntakeWristSubsystem wrist, Direction direction) {
        return switch (direction) {
            case IN -> Commands.run(wrist::holdWristOut, wrist).withName("HoldIntakeWristOutCommand");
            case OUT -> Commands.run(wrist::holdWristIn, wrist).withName("HoldIntakeWristInCommand");
        };
    }

    public static Command intakeDeployCommand(IntakeWristSubsystem wrist, IntakeRollerSubsystem roller) {
        return Commands.parallel(
                new MoveIntakeWristCommand(wrist, Direction.OUT).andThen(holdIntakeWristCommand(wrist, Direction.OUT)),
                new MoveIntakeRollerCommand(roller, Direction.IN).andThen(holdIntakeRollerCommand(roller)));
    }

    public static Command intakeRetractCommand(IntakeWristSubsystem wrist, IntakeRollerSubsystem roller) {
        return Commands.deadline(
                        new MoveIntakeRollerCommand(roller, Direction.OUT),
                        holdIntakeWristCommand(wrist, Direction.OUT))
                .andThen(Commands.parallel(
                        holdIntakeRollerCommand(roller),
                        new MoveIntakeWristCommand(wrist, Direction.IN)
                                .andThen(holdIntakeWristCommand(wrist, Direction.IN))));
    }

    public static Command intakeRollerDefaultCommand(IntakeRollerSubsystem roller) {
        return holdIntakeRollerCommand(roller);
    }

    public static Command intakeWristDefaultCommand(IntakeWristSubsystem wrist) {
        return new MoveIntakeWristCommand(wrist, Direction.IN)
                .andThen(IntakeCommands.holdIntakeWristCommand(wrist, Direction.IN));
    }

    private IntakeCommands() {
        throw new UnsupportedOperationException("IntakeCommands is a static class");
    }
}
