package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.IntakeSubsystem;
import java.util.function.Supplier;

public class TeleopIntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final Supplier<MoInput> inputSupplier;

    public TeleopIntakeCommand(IntakeSubsystem intake, Supplier<MoInput> inputSupplier) {
        this.intake = intake;
        this.inputSupplier = inputSupplier;

        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    /*private void runIntake(boolean runIntake, boolean runIntakeReverse) {
        if (runIntake) {
            intake.wristOut();
            intake.rollerIntake();
        } else if (runIntakeReverse) {
            intake.wristIn();
            intake.rollerShoot();
        }
        else {
            intake.stopMotors();
        }

    }*/

    @Override
    public void execute() {
        intake.rollerShoot();
    }
}
