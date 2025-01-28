package frc.robot.command.groundintake;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.GroundIntakeRollerSubsystem;

public class MoveGroundIntakeRollersCommand extends Command {
    public enum Direction {
        SHOOT,
        INTAKE
    };

    private final GroundIntakeRollerSubsystem intake;
    private final Direction direction;

    public MoveGroundIntakeRollersCommand(GroundIntakeRollerSubsystem intake, Direction direction) {
        this.intake = intake;
        this.direction = direction;

        addRequirements(intake);
    }

    private boolean isBallHeld() {
        // TODO: figure this out later
        return false;
    }

    @Override
    public void execute() {
        if (direction == Direction.SHOOT) {
            intake.rollerShoot();
        } else {
            intake.rollerIntake();
        }
    }

    @Override
    public boolean isFinished() {
        return isBallHeld();
    }
}
