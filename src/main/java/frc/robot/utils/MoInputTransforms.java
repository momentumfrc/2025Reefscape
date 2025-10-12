package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Units;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.input.MoInput;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorMovementRequest;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class MoInputTransforms implements MoInput {
    private final Supplier<MoInput> inputSupplier;
    private final DoubleSupplier maxThrottleSupplier;

    private final VariableSlewRateLimiter driveTranslationLimiter;
    private final VariableSlewRateLimiter driveRotationLimiter;
    private SlewRateLimiter elevatorLimiter;
    private SlewRateLimiter wristLimiter;

    private Vec2 mutMoveRequest = new Vec2(0, 0);

    public MoInputTransforms(
            Supplier<MoInput> inputSupplier,
            Supplier<Double> driveSlewRateSupplier,
            DoubleSupplier maxThrottleSupplier) {
        this.inputSupplier = inputSupplier;
        this.maxThrottleSupplier = maxThrottleSupplier;

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

    private double applyTurnInputTransforms(double value) {
        return MoUtils.curve(MathUtil.applyDeadband(value, MoPrefs.inputDeadzone.get()), MoPrefs.inputTurnCurve.get());
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
        norm *= this.maxThrottleSupplier.getAsDouble();
        mutMoveRequest.scale(norm);

        return mutMoveRequest;
    }

    @Override
    public double getTurnRequest() {
        return applyTurnInputTransforms(
                        driveRotationLimiter.calculate(inputSupplier.get().getTurnRequest()))
                * maxThrottleSupplier.getAsDouble();
    }

    @Override
    public boolean getDriveRobotOriented() {
        return inputSupplier.get().getDriveRobotOriented();
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
    public boolean getReZeroWrist() {
        return inputSupplier.get().getReZeroWrist();
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
        double value = inputSupplier.get().getClimberMoveRequest();
        double maxPwr = MoPrefs.climberMaxPwr.get().in(Units.Value);
        return Math.signum(value) * MathUtil.interpolate(0, maxPwr, Math.abs(value));
    }

    @Override
    public boolean getIntake() {
        return inputSupplier.get().getIntake();
    }

    @Override
    public boolean getIntakeExtakeOverride() {
        return inputSupplier.get().getIntakeExtakeOverride();
    }
}
