package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import java.nio.file.Path;
import java.nio.file.Paths;

public class Constants {
    public static Path DATA_STORE_FILE;

    static {
        if (RobotBase.isReal()) {
            DATA_STORE_FILE = Paths.get("/home/lvuser/pid_constants.ini");
        } else {
            DATA_STORE_FILE = Paths.get("./pid_constants.ini");
        }
    }

    private Constants() {
        throw new UnsupportedOperationException("Constants is a static class");
    }
}
