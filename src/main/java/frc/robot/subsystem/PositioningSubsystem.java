package frc.robot.subsystem;

import com.studica.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Prefs;
import frc.robot.component.LimelightHelpers;
import frc.robot.molib.MoShuffleboard;
import java.util.Map;

public class PositioningSubsystem extends SubsystemBase {
    private static final String LIMELIGHT_ID = "limelight-aprl";

    private Pose2d robotPose = new Pose2d();

    private Field2d field = MoShuffleboard.getInstance().field;

    private GenericEntry didEstablishInitialPosition = MoShuffleboard.getInstance()
            .driveTab
            .add("Initial Position", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();

    private GenericEntry shouldUseAprilTags = MoShuffleboard.getInstance()
            .settingsTab
            .add("Detect AprilTags", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    private GenericEntry tagInView = MoShuffleboard.getInstance().driveTab.add("AprilTag in view", false).getEntry();

    private final AHRS gyro;
    private final DriveSubsystem drive;

    private SwerveDrivePoseEstimator estimator;

    private static enum FieldOrientedDriveMode {
        GYRO,
        ODOMETRY,
        NONE
    };

    private SendableChooser<FieldOrientedDriveMode> fieldOrientedDriveMode =
            MoShuffleboard.enumToChooser(FieldOrientedDriveMode.class);

    private Rotation2d fieldOrientedFwd;

    public PositioningSubsystem(AHRS ahrs, DriveSubsystem drive) {
        this.gyro = ahrs;
        this.drive = drive;

        MoShuffleboard.getInstance().settingsTab.add("Field Oriented Mode", fieldOrientedDriveMode);

        estimator = new SwerveDrivePoseEstimator(
                drive.kinematics, gyro.getRotation2d(), drive.getWheelPositions(), new Pose2d());

        resetFieldOrientedFwd();

        var posGroup = MoShuffleboard.getInstance()
                .driveTab
                .getLayout("Relative Pos", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "RIGHT"));
        posGroup.addDouble("X", () -> robotPose.getX());
        posGroup.addDouble("Y", () -> robotPose.getY());
        posGroup.addDouble("Rot", () -> robotPose.getRotation().getDegrees());
    }

    public Rotation2d getFieldOrientedDriveHeading() {
        var foMode = fieldOrientedDriveMode.getSelected();
        switch (foMode) {
            case GYRO:
                return gyro.getRotation2d().minus(fieldOrientedFwd);
            case ODOMETRY:
                return getRobotPose().getRotation();
            case NONE:
            default:
                return new Rotation2d();
        }
    }

    /**
     * Get robot pose, relative to the origin at the blue alliance.
     */
    public Pose2d getRobotPose() {
        return robotPose;
    }

    public boolean hasInitialPosition() {
        return this.didEstablishInitialPosition.getBoolean(false);
    }

    public void setRobotPose(Pose2d pose) {
        if (this.didEstablishInitialPosition.getBoolean(false)) {
            return;
        }
        this.didEstablishInitialPosition.setBoolean(true);
        this.estimator.resetPosition(gyro.getRotation2d(), drive.getWheelPositions(), pose);
        this.robotPose = pose;
    }

    public void resetFieldOrientedFwd() {
        this.fieldOrientedFwd = gyro.getRotation2d().plus(new Rotation2d(Prefs.navxYawOffset.get()));
    }

    @Override
    public void periodic() {
        // See:
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#using-wpilibs-pose-estimator
        LimelightHelpers.PoseEstimate llPos;
        if (hasInitialPosition()) {
            LimelightHelpers.SetRobotOrientation(
                    LIMELIGHT_ID, estimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            llPos = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_ID);
        } else {
            llPos = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_ID);
        }

        tagInView.setBoolean(llPos != null && llPos.tagCount > 0);

        if (llPos != null
                && Math.abs(gyro.getRate()) < 360
                && llPos.tagCount > 0
                && shouldUseAprilTags.getBoolean(true)) {
            if (!hasInitialPosition()) {
                setRobotPose(llPos.pose);
            } else {
                estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
                estimator.addVisionMeasurement(llPos.pose, llPos.timestampSeconds);
            }
        }

        robotPose = estimator.update(gyro.getRotation2d(), drive.getWheelPositions());
        field.setRobotPose(robotPose);

        if (fieldOrientedDriveMode.getSelected() != FieldOrientedDriveMode.GYRO) resetFieldOrientedFwd();
    }
}
