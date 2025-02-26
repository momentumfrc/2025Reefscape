package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.utils.Vec2;

public class ControllerInput implements MoInput {

    private Joystick driveController = new Joystick(Constants.JOYSTICK.hidport());
    private XboxController elevatorController = new XboxController(Constants.XBOX_CONTROLLER_1.hidport());

    private static final int AUTO_ALIGN_CORAL_LEFT_BUTTON = 8;
    private static final int AUTO_ALIGN_CORAL_RIGHT_BUTTON = 9;
    private static final int AUTO_ALIGN_ALGAE_MIDDLE_BUTTON = 10;

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

    @Override
    public double getClimberMoveRequest() {
        return (-1 * elevatorController.getLeftTriggerAxis()) + elevatorController.getRightTriggerAxis();
    }

    @Override
    // Returns true if the driver is holding the Coral Left auto-align button.
    public boolean getAutoAlignCoralLeft() {
        return driveController.getRawButton(AUTO_ALIGN_CORAL_LEFT_BUTTON);
    }

    @Override
    // Returns true if the driver is holding the Coral Right auto-align button.
    public boolean getAutoAlignCoralRight() {
        return driveController.getRawButton(AUTO_ALIGN_CORAL_RIGHT_BUTTON);
    }

    @Override
    // Returns true if the driver is holding the Algae Middle auto-align button.
    public boolean getAutoAlignAlgaeMiddle() {
        return driveController.getRawButton(AUTO_ALIGN_ALGAE_MIDDLE_BUTTON);
    }
}
