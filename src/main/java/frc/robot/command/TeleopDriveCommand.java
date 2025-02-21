package frc.robot.command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.utils.Vec2;
import java.util.function.Supplier;

public class TeleopDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final PositioningSubsystem positioning;
    private final Supplier<MoInput> inputSupplier;

    public TeleopDriveCommand(DriveSubsystem drive, PositioningSubsystem positioning, Supplier<MoInput> inputSupplier) {
        this.drive = drive;
        this.positioning = positioning;
        this.inputSupplier = inputSupplier;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();

        if (input.getReZeroGyro()) {
            positioning.resetFieldOrientedFwd();
        }

        Vec2 moveRequest = input.getMoveRequest();
        double turnRequest = input.getTurnRequest();

        Rotation2d foHeading = positioning.getFieldOrientedDriveHeading();

        drive.driveCartesian(moveRequest.getY(), moveRequest.getX(), turnRequest, foHeading);
    }
}
