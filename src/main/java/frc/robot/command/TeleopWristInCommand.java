package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.IntakeSubsystem;
import java.util.function.Supplier;

public class TeleopWristInCommand extends Command {
    private final IntakeSubsystem intake;
    private final Supplier<MoInput> inputSupplier;

    public TeleopWristInCommand(IntakeSubsystem intake, Supplier<MoInput> inputSupplier) {
        this.intake = intake;
        this.inputSupplier = inputSupplier;

        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intake.wristIn();
    }
}
