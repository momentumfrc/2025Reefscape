package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.input.MoInput;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorMovementRequest;
import java.util.Optional;
import java.util.function.Supplier;

public class MoInputTransforms implements MoInput {
    private final Supplier<MoInput> inputSupplier;

    private SlewRateLimiter driveTranslationLimiter;
    private SlewRateLimiter driveRotationLimiter;
    private SlewRateLimiter elevatorLimiter;
    private SlewRateLimiter wristLimiter;

    private Vec2 mutMoveRequest = new Vec2(0, 0);

    public MoInputTransforms(Supplier<MoInput> inputSupplier) {
        this.inputSupplier = inputSupplier;

        MoPrefs.driveRampTime.subscribe(
                rampTime -> {
                    double slewRate = 1.0 / rampTime;

                    driveTranslationLimiter = new SlewRateLimiter(slewRate);
                    driveRotationLimiter = new SlewRateLimiter(slewRate);
                },
                true);

        MoPrefs.elevatorRampTime.subscribe(
                rampTime -> {
                    double slewRate = 1.0 / rampTime;

                    elevatorLimiter = new SlewRateLimiter(slewRate);
                    wristLimiter = new SlewRateLimiter(slewRate);
                },
                true);
    }

    private double applyDriveInputTransforms(double value) {
        return MoUtils.curve(MathUtil.applyDeadband(value, MoPrefs.inputDeadzone.get()), MoPrefs.inputCurve.get());
    }

    @Override
    public Optional<ElevatorSetpoint> getElevatorSetpoints() {
        return inputSupplier.get().getElevatorSetpoints();
    }

    @Override
    public Vec2 getMoveRequest() {
        mutMoveRequest.update(inputSupplier.get().getMoveRequest());

        double norm = mutMoveRequest.normalize();
        norm = driveTranslationLimiter.calculate(norm);
        norm = applyDriveInputTransforms(norm);
        mutMoveRequest.scale(norm);

        return mutMoveRequest;
    }

    @Override
    public double getTurnRequest() {
        return applyDriveInputTransforms(
                driveRotationLimiter.calculate(inputSupplier.get().getTurnRequest()));
    }

    @Override
    public boolean getReZeroGyro() {
        return inputSupplier.get().getReZeroGyro();
    }

    @Override
    public ElevatorMovementRequest getElevatorMovementRequest() {
        var requested = inputSupplier.get().getElevatorMovementRequest();
        return new ElevatorMovementRequest(
                elevatorLimiter.calculate(requested.elevatorPower()), wristLimiter.calculate(requested.wristPower()));
    }

    @Override
    public boolean getSaveElevatorSetpoint() {
        return inputSupplier.get().getSaveElevatorSetpoint();
    }

    @Override
    public boolean getReZeroElevator() {
        return inputSupplier.get().getReZeroElevator();
    }

    @Override
    public boolean getEndEffectorIn() {
        return inputSupplier.get().getEndEffectorIn();
    }

    @Override
    public boolean getEndEffectorOut() {
        return inputSupplier.get().getEndEffectorOut();
    }

    @Override
    public double getClimberMoveRequest() {
        return inputSupplier.get().getClimberMoveRequest();
    }

    @Override
    public boolean getIntake() {
        return inputSupplier.get().getIntake();
    }
}
