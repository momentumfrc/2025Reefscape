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

    public static Command idleIntakeRollerCommand(IntakeRollerSubsystem roller) {
        return Commands.run(roller::stopRollerMotor, roller);
    }

    public static Command holdIntakeRollerCommand(IntakeRollerSubsystem roller) {
        return Commands.run(roller::holdBall, roller).withName("HoldIntakeRollerCommand");
    }

    public static Command holdIntakeWristCommand(IntakeWristSubsystem wrist, Direction direction) {
        return switch (direction) {
            case IN -> Commands.run(wrist::holdWristIn, wrist).withName("HoldIntakeWristOutCommand");
            case OUT -> Commands.run(wrist::holdWristOut, wrist).withName("HoldIntakeWristInCommand");
        };
    }

    public static Command softHoldWristForAlgaeIntakeCommand(IntakeWristSubsystem wrist) {
        return Commands.run(wrist::softHoldWristForAlgaeIntake, wrist);
    }

    public static Command intakeDeployCommand(IntakeWristSubsystem wrist, IntakeRollerSubsystem roller) {
        return Commands.parallel(
                        new MoveIntakeWristCommand(wrist, Direction.OUT)
                                .andThen(softHoldWristForAlgaeIntakeCommand(wrist)),
                        new RollerIntakeAlgaeCommand(roller).andThen(holdIntakeRollerCommand(roller)))
                .withName("IntakeWristDeployCommand");
    }

    public static Command intakeRetractCommand(IntakeWristSubsystem wrist, IntakeRollerSubsystem roller) {
        return Commands.deadline(new RollerExtakeAlgaeCommand(roller), holdIntakeWristCommand(wrist, Direction.OUT))
                .andThen(Commands.parallel(
                        idleIntakeRollerCommand(roller),
                        new MoveIntakeWristCommand(wrist, Direction.IN)
                                .andThen(holdIntakeWristCommand(wrist, Direction.IN))))
                .withName("IntakeWristRetractCommand");
    }

    public static Command intakeRollerDefaultCommand(IntakeRollerSubsystem roller) {
        return idleIntakeRollerCommand(roller);
    }

    public static Command intakeWristDefaultCommand(IntakeWristSubsystem wrist) {
        return new MoveIntakeWristCommand(wrist, Direction.IN)
                .andThen(IntakeCommands.holdIntakeWristCommand(wrist, Direction.IN))
                .withName("RetractThenHoldIntakeWristCommand");
    }

    public static Command intakeExtakeOverrideCommand(IntakeWristSubsystem wrist, IntakeRollerSubsystem roller) {
        return Commands.parallel(
                new MoveIntakeWristCommand(wrist, Direction.OUT).andThen(holdIntakeWristCommand(wrist, Direction.OUT)),
                Commands.run(roller::rollerShoot, roller));
    }

    private IntakeCommands() {
        throw new UnsupportedOperationException("IntakeCommands is a static class");
    }
}
