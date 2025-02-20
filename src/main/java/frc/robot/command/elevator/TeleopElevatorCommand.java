package frc.robot.command.elevator;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.component.ElevatorSetpointManager;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.input.MoInput;
import frc.robot.molib.MoShuffleboard;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorMovementRequest;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorPosition;
import java.util.Optional;
import java.util.function.Supplier;

public class TeleopElevatorCommand extends Command {
    private static final ElevatorSetpoint DEFAULT_SETPOINT = ElevatorSetpoint.STOW;

    private final ElevatorSubsystem elevator;
    private final Supplier<MoInput> inputSupplier;

    private boolean smartMotionOverride = false;

    private GenericPublisher setpointPublisher;

    public TeleopElevatorCommand(ElevatorSubsystem elevator, Supplier<MoInput> inputSupplier) {
        this.elevator = elevator;
        this.inputSupplier = inputSupplier;

        setpointPublisher = MoShuffleboard.getInstance()
                .elevatorTab
                .add("Setpoint", "UNKNOWN")
                .getEntry();

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.reZeroWrist();
    }

    private void moveSmartMotion(MoInput input) {
        Optional<ElevatorSetpoint> requestedSetpoint = input.getElevatorSetpoints();
        ElevatorMovementRequest requestedMovement = input.getElevatorMovementRequest();
        boolean shouldSaveSetpoint = input.getSaveElevatorSetpoint();

        if (requestedSetpoint.isPresent()) {
            if (shouldSaveSetpoint
                    && MoShuffleboard.getInstance().tuneSetpointSubscriber.getBoolean(false)) {
                ElevatorSetpointManager.getInstance()
                        .setSetpoint(requestedSetpoint.get(), elevator.getElevatorPosition());
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
            elevator.adjustVelocity(requestedMovement);
        } else {
            setpointPublisher.setString(setpoint.toString());
            elevator.adjustSmartPosition(requestedPosition);
        }
    }

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();
        var controlMode = elevator.controlMode.getSelected();

        if (input.getReZeroElevator()) {
            elevator.reZeroWrist();
        }

        if (controlMode != ElevatorControlMode.SMARTMOTION) {
            setpointPublisher.setString(controlMode.toString());
        }

        switch (controlMode) {
            case FALLBACK_DIRECT_POWER:
                elevator.adjustDirectPower(input.getElevatorMovementRequest());
                return;
            case DIRECT_VELOCITY:
                elevator.adjustVelocity(input.getElevatorMovementRequest());
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
