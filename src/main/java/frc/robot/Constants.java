package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import java.nio.file.Path;
import java.nio.file.Paths;

public class Constants {
    public static final double FLOAT_EPSILON = 1e-6;

    public static final CANAddress SWERVE_TURN_LEFT_FRONT = new CANAddress(11);
    public static final CANAddress SWERVE_TURN_LEFT_REAR = new CANAddress(13);
    public static final CANAddress SWERVE_TURN_RIGHT_FRONT = new CANAddress(5);
    public static final CANAddress SWERVE_TURN_RIGHT_REAR = new CANAddress(12);
    public static final CANAddress SWERVE_DRIVE_LEFT_FRONT = new CANAddress(2);
    public static final CANAddress SWERVE_DRIVE_LEFT_REAR = new CANAddress(3);
    public static final CANAddress SWERVE_DRIVE_RIGHT_FRONT = new CANAddress(1);
    public static final CANAddress SWERVE_DRIVE_RIGHT_REAR = new CANAddress(4);
    public static final CANAddress ELEVATORA = new CANAddress(4);
    public static final CANAddress ELEVATORB = new CANAddress(9);
    public static final CANAddress ELEVATOR_WRIST = new CANAddress(2);
    public static final CANAddress END_EFFECTOR = new CANAddress(0);

    public static final HIDPort JOYSTICK = new HIDPort(0);
    public static final HIDPort XBOX_CONTROLLER_1 = new HIDPort(1);

    public static Path DATA_STORE_FILE;

    static {
        if (RobotBase.isReal()) {
            DATA_STORE_FILE = Paths.get("/home/lvuser/pid_constants.ini");
        } else {
            DATA_STORE_FILE = Paths.get("./pid_constants.ini");
        }
    }

    /**
     * A wrapper class holding a single integer value representing a CAN Address. The point of this
     * class is to indicate that the wrapped value is a CAN Address in a more robust way than just
     * adding "CAN_ADDR" to the constant's name.
     */
    public static record CANAddress(int address) {}

    /**
     * A wrapper class holding a single integer value representing a HID Port. The point of this
     * class is to indicate that the wrapped value is a HID Port in a more robust way than just
     * adding "PORT" to the constant's name.
     */
    public static record HIDPort(int hidport) {}

    private Constants() {
        throw new UnsupportedOperationException("Constants is a static class");
    }
}
