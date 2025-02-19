package frc.robot.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.utils.MoUtils;
import frc.robot.utils.Vec2;
import java.util.function.Supplier;

public class TeleopDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final PositioningSubsystem positioning;
    private final Supplier<MoInput> inputSupplier;

    private SlewRateLimiter translationLimiter;
    private SlewRateLimiter rotationLimiter;

    private Vec2 mutMoveRequest = new Vec2(0, 0);

    public TeleopDriveCommand(DriveSubsystem drive, PositioningSubsystem positioning, Supplier<MoInput> inputSupplier) {
        this.drive = drive;
        this.positioning = positioning;
        this.inputSupplier = inputSupplier;

        MoPrefs.driveRampTime.subscribe(
                rampTime -> {
                    double slewRate = 1.0 / rampTime;

                    translationLimiter = new SlewRateLimiter(slewRate);
                    rotationLimiter = new SlewRateLimiter(slewRate);
                },
                true);

        addRequirements(drive);
    }

    private double applyInputTransforms(double value) {
        return MoUtils.curve(MathUtil.applyDeadband(value, MoPrefs.inputDeadzone.get()), MoPrefs.inputCurve.get());
    }

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();

        if (input.getReZeroGyro()) {
            positioning.resetFieldOrientedFwd();
        }

        mutMoveRequest.update(input.getMoveRequest());
        double norm = mutMoveRequest.normalize();
        norm = applyInputTransforms(norm);
        norm = translationLimiter.calculate(norm);
        mutMoveRequest.scale(norm);

        double turnRequest = input.getTurnRequest();
        turnRequest = applyInputTransforms(turnRequest);

        Rotation2d foHeading = positioning.getFieldOrientedDriveHeading();

        drive.driveCartesian(mutMoveRequest.getY(), mutMoveRequest.getX(), turnRequest, foHeading);
    }
}
