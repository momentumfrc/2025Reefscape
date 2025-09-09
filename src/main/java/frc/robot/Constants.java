package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.DriverStation;
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
    public static final CANAddress END_EFFECTOR = new CANAddress(10);

    public static final CANAddress CLIMBER_LEFT = new CANAddress(21);
    public static final CANAddress CLIMBER_RIGHT = new CANAddress(3);
    public static final PWMPort CLIMBER_RACHET = new PWMPort(0);

    public static final CANAddress INTAKE_WRIST = new CANAddress(7);
    public static final CANAddress INTAKE_ROLLER = new CANAddress(14);

    public static final HIDPort JOYSTICK = new HIDPort(0);
    public static final HIDPort XBOX_CONTROLLER_1 = new HIDPort(1);

    public static final PWMPort leds = new PWMPort(1);
    public static final PWMPort CAGE_SENSOR = new PWMPort(9);

    public static final int END_EFFECTOR_ROLLERS_PDH_PORT = 15;

    public static Path DATA_STORE_FILE;

    public static RobotConfig pathPlannerRobotConfig;

    static {
        if (RobotBase.isReal()) {
            DATA_STORE_FILE = Paths.get("/home/lvuser/pid_constants.ini");
        } else {
            DATA_STORE_FILE = Paths.get("./pid_constants.ini");
        }

        try {
            pathPlannerRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            DriverStation.reportError("Could not load PP robot config", e.getStackTrace());
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

    public static record PWMPort(int port) {}

    private Constants() {
        throw new UnsupportedOperationException("Constants is a static class");
    }
}
