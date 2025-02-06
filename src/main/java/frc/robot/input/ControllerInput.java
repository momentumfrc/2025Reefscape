package frc.robot.input;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorMovementRequest;
import frc.robot.utils.Vec2;
import java.util.Optional;

public class ControllerInput implements MoInput {

    private XboxController controller = new XboxController(Constants.XBOX_CONTROLLER_1.hidport());

    private Vec2 moveRequest = new Vec2(0, 0);

    @Override
    public Vec2 getMoveRequest() {
        moveRequest.update(controller.getLeftX(), controller.getLeftY());
        return moveRequest;
    }

    @Override
    public double getTurnRequest() {
        return -1 * controller.getRightX();
    }

    @Override
    public boolean getReZeroGyro() {
        return controller.getBackButtonPressed();
    }

    @Override
    public boolean getSaveElevatorSetpoint() {
        return controller.getStartButtonPressed();
    }

    @Override
    public ElevatorMovementRequest getElevatorMovementRequest() {
        if (controller.getLeftBumperButton()) {
            return new ElevatorMovementRequest(controller.getLeftY(), controller.getRightY());
        } else return new ElevatorMovementRequest(0, 0);
    }

    @Override
    public Optional<ElevatorSetpoint> getElevatorSetpoints() {
        if (controller.getBButton()) {
            return Optional.of(ElevatorSetpoint.STOW);
        } else if (controller.getAButton()) {
            return Optional.of(ElevatorSetpoint.PROCESSOR);
        } else if (controller.getYButton()) {
            return Optional.of(ElevatorSetpoint.L3);
        }

        return Optional.empty();
    }

    @Override
    public boolean getReZeroElevator() {
        return controller.getStartButton();
    }
}
