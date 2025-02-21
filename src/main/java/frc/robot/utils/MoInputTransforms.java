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

    private VariableSlewRateLimiter driveTranslationLimiter;
    private VariableSlewRateLimiter driveRotationLimiter;
    private SlewRateLimiter elevatorLimiter;
    private SlewRateLimiter wristLimiter;

    private Vec2 mutMoveRequest = new Vec2(0, 0);

    public MoInputTransforms(Supplier<MoInput> inputSupplier, Supplier<Double> driveSlewRateSupplier) {
        this.inputSupplier = inputSupplier;

        driveTranslationLimiter = new VariableSlewRateLimiter(driveSlewRateSupplier);
        driveRotationLimiter = new VariableSlewRateLimiter(driveSlewRateSupplier);

        MoPrefs.elevatorRampTime.subscribe(
                rampTime -> {
                    double slewRate = 1.0 / rampTime;

                    elevatorLimiter = new SlewRateLimiter(slewRate);
                    wristLimiter = new SlewRateLimiter(slewRate);
                },
                true);
    }

    private double applyInputTransforms(double value) {
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
        norm = applyInputTransforms(norm);
        norm = driveTranslationLimiter.calculate(norm);
        mutMoveRequest.scale(norm);

        return mutMoveRequest;
    }

    @Override
    public double getTurnRequest() {
        return applyInputTransforms(
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
                elevatorLimiter.calculate(applyInputTransforms(requested.elevatorPower())),
                wristLimiter.calculate(applyInputTransforms(requested.wristPower())));
    }

    @Override
    public boolean getSaveElevatorSetpoint() {
        return inputSupplier.get().getSaveElevatorSetpoint();
    }

    @Override
    public boolean getRunSysid() {
        return inputSupplier.get().getRunSysid();
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
