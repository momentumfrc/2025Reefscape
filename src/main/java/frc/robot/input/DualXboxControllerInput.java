package frc.robot.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorMovementRequest;
import frc.robot.utils.Vec2;
import java.util.Optional;

public class DualXboxControllerInput implements MoInput {

    private XboxController driveController = new XboxController(Constants.JOYSTICK.hidport());
    private XboxController elevatorController = new XboxController(Constants.XBOX_CONTROLLER_1.hidport());

    private Vec2 moveRequest = new Vec2(0, 0);

    private double throttle = 0;

    private double getThrottle() {
        if (driveController.getAButtonPressed()) throttle -= .05;
        if (driveController.getYButtonPressed()) throttle += .05;
        MathUtil.clamp(throttle, 0, 1);
        return throttle;
    }

    @Override
    public Vec2 getMoveRequest() {
        moveRequest.update(driveController.getLeftX(), driveController.getLeftY());
        moveRequest.scale(getThrottle());
        return moveRequest;
    }

    @Override
    public double getTurnRequest() {
        return -1 * driveController.getRightX() * getThrottle();
    }

    @Override
    public boolean getReZeroGyro() {
        return driveController.getLeftBumperButton();
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
                return Optional.of(ElevatorSetpoint.L3_CORAL);
            } else if (elevatorController.getBButton()) {
                return Optional.of(ElevatorSetpoint.L2_CORAL);
            } else if (elevatorController.getAButton()) {
                return Optional.of(ElevatorSetpoint.L1_CORAL);
            }
        } else {
            if (elevatorController.getYButton()) {
                return Optional.of(ElevatorSetpoint.CORAL_STATION);
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
    public boolean getRunSysid() {
        return elevatorController.getStartButton();
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
        return driveController.getRightBumperButton();
    }

    @Override
    public double getClimberMoveRequest() {
        return (-1 * elevatorController.getLeftTriggerAxis()) + elevatorController.getRightTriggerAxis();
    }

    @Override
    public boolean getIntakeExtakeOverride() {
        return false;
    }
}
