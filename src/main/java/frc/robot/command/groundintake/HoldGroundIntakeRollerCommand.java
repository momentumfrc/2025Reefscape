package frc.robot.command.groundintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.GroundIntakeRollerSubsystem;

public class HoldGroundIntakeRollerCommand extends Command {
    private final GroundIntakeRollerSubsystem intakeSubsystem;

    public HoldGroundIntakeRollerCommand(GroundIntakeRollerSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.stopRollerMotor();
    }
}
