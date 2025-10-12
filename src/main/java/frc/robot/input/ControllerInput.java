package frc.robot.input;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorMovementRequest;
import frc.robot.utils.Vec2;
import java.util.Optional;

public class ControllerInput implements MoInput {

    private Joystick driveController = new Joystick(Constants.JOYSTICK.hidport());
    private XboxController elevatorController = new XboxController(Constants.XBOX_CONTROLLER_1.hidport());

    private Vec2 moveRequest = new Vec2(0, 0);

    private double getThrottle() {
        double throttle = ((-1 * driveController.getThrottle()) + 1) / 2;

        int pov = driveController.getPOV();
        if (pov >= 90 && pov <= 270) {
            throttle = Math.min(MoPrefs.driveSlowSpeed.get().in(Units.Value), throttle);
        }

        return throttle;
    }

    @Override
    public Vec2 getMoveRequest() {
        moveRequest.update(driveController.getX(), driveController.getY());
        moveRequest.scale(getThrottle());
        return moveRequest;
    }

    @Override
    public boolean getDriveRobotOriented() {
        return driveController.getRawButton(3);
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
        return elevatorController.getStartButton();
    }

    @Override
    public ElevatorMovementRequest getElevatorMovementRequest() {
        return new ElevatorMovementRequest(-1 * elevatorController.getLeftY(), elevatorController.getRightY());
    }

    @Override
    public Optional<ElevatorSetpoint> getElevatorSetpoints() {
        double pov = elevatorController.getPOV();
        if (pov > 90 && pov < 270) {
            if (elevatorController.getYButton()) {
                return Optional.of(ElevatorSetpoint.L3_ALGAE);
            } else if (elevatorController.getBButton()) {
                return Optional.of(ElevatorSetpoint.L2_ALGAE);
            } else if (elevatorController.getAButton()) {
                return Optional.of(ElevatorSetpoint.L1_ALGAE);
            }
        } else if (pov >= 0) {
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
    public boolean getReZeroWrist() {
        return elevatorController.getBackButtonPressed();
    }

    @Override
    public boolean getReZeroElevator() {
        return elevatorController.getStartButton();
    }

    @Override
    public boolean getRunSysid() {
        return false;
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
        double input = 0;
        if (driveController.getRawButton(11)) {
            input += 1;
        }
        if (driveController.getRawButton(12)) {
            input -= 1;
        }

        input += (-1 * elevatorController.getLeftTriggerAxis()) + elevatorController.getRightTriggerAxis();

        return input;
    }

    @Override
    public boolean getIntakeExtakeOverride() {
        return driveController.getRawButton(2);
    }
}
