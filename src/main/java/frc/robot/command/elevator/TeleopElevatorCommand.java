package frc.robot.command.elevator;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Prefs;
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
    private ElevatorSetpoint lastSetpoint = ElevatorSetpoint.STOW;

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

    private void movePosition(MoInput input) {
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

        ElevatorSetpoint setpoint = requestedSetpoint.orElse(lastSetpoint);
        lastSetpoint = setpoint;
        ElevatorPosition requestedPosition =
                ElevatorSetpointManager.getInstance().getSetpoint(setpoint);

        double varianceThresh = Prefs.elevatorSetpointVarianceThreshold.get().in(Units.Value);

        ElevatorPosition coralStationPosition =
                ElevatorSetpointManager.getInstance().getSetpoint(ElevatorSetpoint.CORAL_STATION);
        ElevatorPosition stowPosition = ElevatorSetpointManager.getInstance().getSetpoint(ElevatorSetpoint.STOW);

        if (setpoint == ElevatorSetpoint.CORAL_STATION) {
            if (!elevator.getElevatorHeight().isNear(coralStationPosition.elevatorDistance(), varianceThresh)) {
                requestedPosition =
                        new ElevatorPosition(coralStationPosition.elevatorDistance(), stowPosition.wristAngle());
            } else if (elevator.isWristNominalReverseLimitEnabled()) {
                elevator.disableWristNominalReverseLimit();
            }
        } else {
            if (elevator.isWristInDanger()) {
                if (elevator.getElevatorHeight().isNear(coralStationPosition.elevatorDistance(), varianceThresh)) {
                    requestedPosition =
                            new ElevatorPosition(coralStationPosition.elevatorDistance(), stowPosition.wristAngle());
                } else {
                    DriverStation.reportError("Wrist in danger yet elevator is not at coral station height!", false);
                    requestedPosition = new ElevatorPosition(elevator.getElevatorHeight(), stowPosition.wristAngle());
                }
            }
            if (!elevator.isWristNominalReverseLimitEnabled()) {
                elevator.enableWristNominalReverseLimit();
            }
        }

        if (smartMotionOverride) {
            setpointPublisher.setString("OVERRIDE");
            elevator.adjustVelocity(checkLimits(requestedMovement));
        } else {
            setpointPublisher.setString(setpoint.toString());
            elevator.adjustPosition(requestedPosition);
        }
    }

    public ElevatorMovementRequest checkLimits(ElevatorMovementRequest request) {
        double varianceThresh = Prefs.elevatorSetpointVarianceThreshold.get().in(Units.Value);

        ElevatorPosition coralStationPosition =
                ElevatorSetpointManager.getInstance().getSetpoint(ElevatorSetpoint.CORAL_STATION);
        if (elevator.getElevatorHeight().isNear(coralStationPosition.elevatorDistance(), varianceThresh)) {
            if (elevator.isWristNominalReverseLimitEnabled()) {
                elevator.disableWristNominalReverseLimit();
            }
        } else {
            if (!elevator.isWristNominalReverseLimitEnabled()) {
                elevator.enableWristNominalReverseLimit();
            }
        }

        if (elevator.isWristInDanger()) {
            return new ElevatorMovementRequest(0, request.wristPower());
        }

        return request;
    }

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();
        var controlMode = elevator.controlMode.getSelected();

        if (input.getReZeroElevator()) {
            elevator.reZeroWrist();
        }

        if (controlMode == ElevatorControlMode.SMARTMOTION && !elevator.hasZero()) {
            DriverStation.reportWarning("Disabling smartmotion because the elevator is not reliably zeroed", false);
            controlMode = ElevatorControlMode.DIRECT_VELOCITY;
        }

        if (controlMode != ElevatorControlMode.SMARTMOTION) {
            setpointPublisher.setString(controlMode.toString());
        }

        switch (controlMode) {
            case FALLBACK_DIRECT_POWER:
                elevator.adjustDirectPower(checkLimits(input.getElevatorMovementRequest()));
                return;
            case DIRECT_VELOCITY:
                elevator.adjustVelocity(checkLimits(input.getElevatorMovementRequest()));
                return;
            case SMARTMOTION:
            case RAW_POSITION_PID:
                movePosition(input);
                return;
        }
    }

    @Override
    public void end(boolean interrupted) {
        setpointPublisher.setString("UNKNOWN");
    }
}
