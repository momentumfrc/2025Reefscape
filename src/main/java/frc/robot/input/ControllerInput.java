package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorMovementRequest;
import frc.robot.utils.Vec2;
import java.util.Optional;

public class ControllerInput implements MoInput {

    private Joystick driveController = new Joystick(Constants.JOYSTICK.hidport());
    private XboxController elevatorController = new XboxController(Constants.XBOX_CONTROLLER_1.hidport());

    private Vec2 moveRequest = new Vec2(0, 0);

    private double getThrottle() {
        return ((-1 * driveController.getThrottle()) + 1) / 2;
    }

    @Override
    public Vec2 getMoveRequest() {
        moveRequest.update(driveController.getX(), driveController.getY());
        moveRequest.scale(getThrottle());
        return moveRequest;
    }

    @Override
    public double getTurnRequest() {
        return -1 * driveController.getZ() * getThrottle();
    }

    @Override
    public boolean getReZeroGyro() {
        return driveController.getRawButton(7);
    }

    @Override
    public boolean getSaveElevatorSetpoint() {
        return elevatorController.getStartButtonPressed();
    }

    @Override
    public ElevatorMovementRequest getElevatorMovementRequest() {
        return new ElevatorMovementRequest(elevatorController.getLeftY(), elevatorController.getRightY());
    }

    @Override
    public Optional<ElevatorSetpoint> getElevatorSetpoints() {
        double pov = elevatorController.getPOV();
        if (pov == 0) {
            if (elevatorController.getYButton()) {
                return Optional.of(ElevatorSetpoint.L3);
            } else if (elevatorController.getBButton()) {
                return Optional.of(ElevatorSetpoint.L2);
            } else if (elevatorController.getAButton()) {
                return Optional.of(ElevatorSetpoint.L1);
            }
        } else {
            if (elevatorController.getYButton()) {
                return Optional.of(ElevatorSetpoint.INTAKE);
            } else if (elevatorController.getBButton()) {
                return Optional.of(ElevatorSetpoint.STOW);
            } else if (elevatorController.getAButton()) {
                return Optional.of(ElevatorSetpoint.PROCESSOR);
            }
        }
        return Optional.empty();
    }

    @Override
    public boolean getReZeroElevator() {
        return elevatorController.getBackButtonPressed();
    }

    @Override
    public boolean getEndEffectorIn() {
        return elevatorController.getLeftBumperButton();
    }

    @Override
    public boolean getEndEffectorOut() {
        return elevatorController.getRightBumperButton();
    }

    public boolean getIntake() {
        return driveController.getRawButton(1);
    }

    @Override
    public double getClimberMoveRequest() {
        return (-1 * elevatorController.getLeftTriggerAxis()) + elevatorController.getRightTriggerAxis();
    }
}
