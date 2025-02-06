package frc.robot.command.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.IntakeRollerSubsystem;

public class HoldIntakeRollerCommand extends Command {
    private final IntakeRollerSubsystem intakeSubsystem;

    public HoldIntakeRollerCommand(IntakeRollerSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.stopRollerMotor();
    }
}
