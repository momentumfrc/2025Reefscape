package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.IntakeSubsystem;
import java.util.function.Supplier;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

public class TeleopIntakeCommand extends Command {
    public enum Direction {
        SHOOT,
        INTAKE
    };

    private final IntakeSubsystem intake;
    private final Direction direction;

    public TeleopIntakeCommand(IntakeSubsystem intake, Direction direction) {
        this.intake = intake;
        this.direction = direction;

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
        if(direction == Direction.SHOOT) {
            intake.rollerShoot();
        } else {
            intake.rollerIntake();
        }
    }
}
