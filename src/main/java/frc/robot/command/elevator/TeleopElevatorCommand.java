package frc.robot.command.elevator;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.ElevatorSetpointManager;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.input.MoInput;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorMovementRequest;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorPosition;
import java.util.Optional;
import java.util.function.Supplier;

public class TeleopElevatorCommand extends Command {
    private static final double SHOULDER_HARD_STOP_OVERRIDE_TIMEOUT = 1;

    private static final ElevatorSetpoint DEFAULT_SETPOINT = ElevatorSetpoint.STOW;

    private final ElevatorSubsystem elevators;
    private final Supplier<MoInput> inputSupplier;

    private boolean smartMotionOverride = false;

    private SlewRateLimiter elevatorLimiter;
    private SlewRateLimiter wristLimiter;

    private GenericPublisher setpointPublisher;

    private boolean zeroElevatorPressed = false;
    private boolean elevatorLimitOverride = false;
    private Timer elevatorHardStopOverrideTimer = new Timer();

    public TeleopElevatorCommand(ElevatorSubsystem elevators, Supplier<MoInput> inputSupplier) {
        this.elevators = elevators;
        this.inputSupplier = inputSupplier;

        MoPrefs.elevatorRampTime.subscribe(
                rampTime -> {
                    double slewRate = 1.0 / rampTime;

                    elevatorLimiter = new SlewRateLimiter(slewRate);
                    wristLimiter = new SlewRateLimiter(slewRate);
                },
                true);

        setpointPublisher = MoShuffleboard.getInstance()
                .elevatorTab
                .add("Setpoint", "UNKNOWN")
                .getEntry();

        addRequirements(elevators);
    }

    @Override
    public void initialize() {
        zeroElevatorPressed = false;
        elevatorLimitOverride = false;
        elevatorHardStopOverrideTimer.restart();
        elevators.reZeroElevator();
    }

    private ElevatorMovementRequest getMovementRequest(MoInput input) {
        ElevatorMovementRequest requestedMovement = input.getElevatorMovementRequest();
        return new ElevatorMovementRequest(
                elevatorLimiter.calculate(requestedMovement.elevatorPower()),
                wristLimiter.calculate(requestedMovement.wristPower()));
    }

    private void moveSmartMotion(MoInput input) {
        Optional<ElevatorSetpoint> requestedSetpoint = input.getElevatorSetpoints();
        ElevatorMovementRequest requestedMovement = getMovementRequest(input);
        boolean shouldSaveSetpoint = input.getSaveElevatorSetpoint();

        if (requestedSetpoint.isPresent()) {
            if (shouldSaveSetpoint
                    && MoShuffleboard.getInstance().tuneSetpointSubscriber.getBoolean(false)) {
                ElevatorSetpointManager.getInstance()
                        .setSetpoint(requestedSetpoint.get(), elevators.getElevatorPosition());
            } else {
                smartMotionOverride = false;
            }
        }

        if (!requestedMovement.isZero()) {
            smartMotionOverride = true;
        }

        ElevatorSetpoint setpoint = requestedSetpoint.orElse(DEFAULT_SETPOINT);
        ElevatorPosition requestedPosition =
                ElevatorSetpointManager.getInstance().getSetpoint(setpoint);
        if (smartMotionOverride) {
            setpointPublisher.setString("OVERRIDE");
            elevators.adjustVelocity(requestedMovement);
        } else {
            setpointPublisher.setString(setpoint.toString());
            elevators.adjustSmartPosition(requestedPosition);
        }
    }

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();
        var controlMode = elevators.controlMode.getSelected();

        // When re-zero is held, we disable the reverse limit. This is to account for if the robot was turned on with
        // the elevator up, so the driver can force it down to its proper zeroing position.
        // Then, when re-zero is released, the zeroing actually happens.
        if (input.getReZeroElevator()) {
            zeroElevatorPressed = true;
            if (elevatorHardStopOverrideTimer.hasElapsed(SHOULDER_HARD_STOP_OVERRIDE_TIMEOUT)) {
                if (!elevatorLimitOverride) {
                    elevatorLimitOverride = true;
                    elevators.disableWristReverseLimit();
                }
            }
        } else {
            if (zeroElevatorPressed) {
                elevators.enableWristReverseLimit();
                elevatorLimitOverride = false;
                elevators.reZeroElevator();
            }
            zeroElevatorPressed = false;
            elevatorHardStopOverrideTimer.restart();
        }

        if (controlMode != ElevatorControlMode.SMARTMOTION) {
            setpointPublisher.setString(controlMode.toString());
        }

        switch (controlMode) {
            case FALLBACK_DIRECT_POWER:
                elevators.adjustDirectPower(getMovementRequest(input));
                return;
            case DIRECT_VELOCITY:
                elevators.adjustVelocity(getMovementRequest(input));
                return;
            case SMARTMOTION:
                moveSmartMotion(input);
                return;
        }
    }

    @Override
    public void end(boolean interrupted) {
        setpointPublisher.setString("UNKNOWN");
    }
}
