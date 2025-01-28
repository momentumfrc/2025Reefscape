package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.utils.Vec2;

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
    public boolean getIntake() {
        return driveController.getRawButton(1);
    }
}
