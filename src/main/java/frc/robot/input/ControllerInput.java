package frc.robot.input;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.utils.Vec2;

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
    public boolean getIntakeIn() {
        return controller.getAButtonPressed();
    }

    @Override
    public boolean getIntakeShoot() {
        return controller.getBButtonPressed();
    }

    @Override
    public boolean getIntakeAlgae() {
        return controller.getXButtonPressed();
    }

    @Override
    public boolean getIntakeOut() {
        return controller.getYButtonPressed();
    }
}
