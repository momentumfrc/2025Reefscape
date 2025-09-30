package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Prefs;
import frc.robot.component.SwerveModule;
import frc.robot.molib.MoShuffleboard;
import frc.robot.utils.MutablePIDConstants;
import frc.robot.utils.TunerUtils;
import java.util.Map;
import java.util.function.Consumer;

public class DriveSubsystem extends SubsystemBase {

    /**
     * How frequently the absolute encoder will be used to re-zero the relative encoders.
     */
    private static final Time RESET_ENCODER_PERIOD = Units.Seconds.of(0.5);

    /**
     * The distance along one axis, in meters, from the center of the robot to the center of a
     * swerve module wheel. Since the chassis is square, we don't have to keep track of the X and Y
     * dimensions separately.
     * <p>
     * The robot frame has a side length of 29.5". The mounting holes for the serve modules
     * are 0.5" inset from the edge of the frame. Per the SDS layout diagram of the MK4 swerve
     * modules, the length between the mounting holes and the center of the swerve wheel is 2.75".
     * So, the distance from the center of the robot to the center of a swerve wheel is
     * 29.5"/2 - 0.5" - 2.75" = 11.5".
     */
    public static final Distance SWERVE_WHEEL_OFFSET = Units.Inch.of(11.5);

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule rearLeft;
    private final SwerveModule rearRight;

    private final Timer resetEncoderTimer = new Timer();

    public final SwerveDriveKinematics kinematics;

    private final MutablePIDConstants translationPIDConstants = new MutablePIDConstants();
    private final MutablePIDConstants rotationPIDConstants = new MutablePIDConstants();

    public DriveSubsystem() {
        super("Drive");

        this.frontLeft = new SwerveModule(
                "SWRV FL",
                new SparkMax(Constants.SWERVE_TURN_LEFT_FRONT.address(), MotorType.kBrushless),
                new TalonFX(Constants.SWERVE_DRIVE_LEFT_FRONT.address()),
                Prefs.swerveFLZero,
                Prefs.swerveRotScale,
                Prefs.swerveDistScale);

        this.frontRight = new SwerveModule(
                "SWRV FR",
                new SparkMax(Constants.SWERVE_TURN_RIGHT_FRONT.address(), MotorType.kBrushless),
                new TalonFX(Constants.SWERVE_DRIVE_RIGHT_FRONT.address()),
                Prefs.swerveFRZero,
                Prefs.swerveRotScale,
                Prefs.swerveDistScale);

        this.rearLeft = new SwerveModule(
                "SWRV RL",
                new SparkMax(Constants.SWERVE_TURN_LEFT_REAR.address(), MotorType.kBrushless),
                new TalonFX(Constants.SWERVE_DRIVE_LEFT_REAR.address()),
                Prefs.swerveRLZero,
                Prefs.swerveRotScale,
                Prefs.swerveDistScale);

        this.rearRight = new SwerveModule(
                "SWRV RR",
                new SparkMax(Constants.SWERVE_TURN_RIGHT_REAR.address(), MotorType.kBrushless),
                new TalonFX(Constants.SWERVE_DRIVE_RIGHT_REAR.address()),
                Prefs.swerveRRZero,
                Prefs.swerveRotScale,
                Prefs.swerveDistScale);

        resetEncoderTimer.start();

        double offset_meters = SWERVE_WHEEL_OFFSET.in(Units.Meters);
        Translation2d fl = new Translation2d(offset_meters, offset_meters);
        Translation2d fr = new Translation2d(offset_meters, -offset_meters);
        Translation2d rl = new Translation2d(-offset_meters, offset_meters);
        Translation2d rr = new Translation2d(-offset_meters, -offset_meters);

        this.kinematics = new SwerveDriveKinematics(fl, fr, rl, rr);

        forEachSwerveModule((module) -> {
            var group = MoShuffleboard.getInstance()
                    .driveTab
                    .getLayout(module.getKey(), BuiltInLayouts.kList)
                    .withSize(2, 2)
                    .withProperties(Map.of("Label Position", "RIGHT"));
            group.addDouble(
                            "Drive Position (raw)",
                            () -> module.driveMotor.getRotorPosition().getValueAsDouble())
                    .withWidget(BuiltInWidgets.kTextView);
            group.addDouble(
                            "Drive Position (m)",
                            () -> module.distEncoder.getPosition().in(Units.Meters))
                    .withWidget(BuiltInWidgets.kTextView);
            group.addDouble(
                            "Drive Velocity (m_s)",
                            () -> module.distEncoder.getVelocity().in(Units.MetersPerSecond))
                    .withWidget(BuiltInWidgets.kTextView);
            group.addDouble(
                            "Turn Relative (R)",
                            () -> module.relativeEncoder.getPosition().in(Units.Rotations))
                    .withWidget(BuiltInWidgets.kTextView);
            group.addDouble(
                            "Turn Absolute (R)",
                            () -> module.absoluteEncoder.getPosition().in(Units.Rotations))
                    .withWidget(BuiltInWidgets.kTextView);
        });

        TunerUtils.forPathPlanner(translationPIDConstants, "PP trans PID");
        TunerUtils.forPathPlanner(rotationPIDConstants, "PP rot PID");

        MoShuffleboard.getInstance().driveTab.add(this);
    }

    public SwerveModulePosition[] getWheelPositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()
        };
    }

    private void forEachSwerveModule(Consumer<SwerveModule> forBody) {
        forBody.accept(frontLeft);
        forBody.accept(frontRight);
        forBody.accept(rearLeft);
        forBody.accept(rearRight);
    }

    /**
     * Robot-oriented cartesian drive
     */
    public void driveCartesian(double fwdRequest, double leftRequest, double turnRequest) {
        this.driveCartesian(fwdRequest, leftRequest, turnRequest, new Rotation2d());
    }

    private MutLinearVelocity mutFwdRequest = Units.MetersPerSecond.mutable(0);
    private MutLinearVelocity mutLeftRequest = Units.MetersPerSecond.mutable(0);
    private MutAngularVelocity mutTurnRequest = Units.RotationsPerSecond.mutable(0);

    /**
     * Field-oriented cartesian drive
     */
    public void driveCartesian(
            double fwdRequest, double leftRequest, double turnRequest, Rotation2d fieldOrientedDriveAngle) {
        var maxLinearSpeed = Prefs.swerveMaxLinearSpeed.get();
        var maxAngularSpeed = Prefs.swerveMaxAngularSpeed.get();

        mutFwdRequest.mut_replace(maxLinearSpeed);
        mutLeftRequest.mut_replace(maxLinearSpeed);
        mutTurnRequest.mut_replace(maxAngularSpeed);

        mutFwdRequest.mut_times(fwdRequest);
        mutLeftRequest.mut_times(leftRequest);
        mutTurnRequest.mut_times(turnRequest);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                mutFwdRequest, mutLeftRequest, mutTurnRequest, fieldOrientedDriveAngle);

        driveRobotRelativeSpeeds(speeds);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(
                frontLeft.getState(), frontRight.getState(),
                rearLeft.getState(), rearRight.getState());
    }

    public void driveRobotRelativeSpeeds(ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFeedforwards) {
        driveRobotRelativeSpeeds(chassisSpeeds);
    }

    public void driveRobotRelativeSpeeds(ChassisSpeeds speeds) {
        driveSwerveStates(kinematics.toSwerveModuleStates(speeds));
    }

    public PPHolonomicDriveController getDriveController() {
        return new PPHolonomicDriveController(
                translationPIDConstants.toImmutable(), rotationPIDConstants.toImmutable());
    }

    public void driveSwerveStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Prefs.swerveMaxLinearSpeed.get());

        frontLeft.drive(states[0]);
        frontRight.drive(states[1]);
        rearLeft.drive(states[2]);
        rearRight.drive(states[3]);
    }

    public void resetRelativeEncoders() {
        frontLeft.setRelativePosition();
        frontRight.setRelativePosition();
        rearLeft.setRelativePosition();
        rearRight.setRelativePosition();
    }

    @Override
    public void periodic() {
        if (resetEncoderTimer.advanceIfElapsed(RESET_ENCODER_PERIOD.in(Units.Seconds))) {
            resetRelativeEncoders();
        }

        forEachSwerveModule(module -> module.turnMotorConfig.checkForBrownout());
    }
}
