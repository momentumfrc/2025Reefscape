package frc.robot.command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.IntakeSubsystem;

public class TeleopShootCommand extends Command{
    private final IntakeSubsystem intake;
    private final Supplier<MoInput> inputSupplier;

    public TeleopShootCommand(IntakeSubsystem intake, Supplier<MoInput> inputSupplier) {
        this.intake = intake;
        this.inputSupplier = inputSupplier;
        addRequirements(intake);
    }
    @Override
    public void initialize() {

    }

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
