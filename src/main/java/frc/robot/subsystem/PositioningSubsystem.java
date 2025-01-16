package frc.robot.subsystem;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.molib.prefs.MoPrefs;

public class PositioningSubsystem extends SubsystemBase {

    private final AHRS gyro;

    private Rotation2d fieldOrientedFwd;

    public PositioningSubsystem(AHRS gyro) {
        this.gyro = gyro;

        resetFieldOrientedFwd();
    }

    public Rotation2d getFieldOrientedDriveHeading() {
        return gyro.getRotation2d().minus(fieldOrientedFwd);
    }

    public void resetFieldOrientedFwd() {
        this.fieldOrientedFwd = gyro.getRotation2d().plus(new Rotation2d(MoPrefs.navxYawOffset.get()));
    }

    public Pose2d getRobotPose() {
        // TODO
        return null;
    }
}
