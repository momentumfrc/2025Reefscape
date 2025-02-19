package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
<<<<<<< HEAD
import com.kauailabs.navx.frc.AHRS;
import com.momentum4999.motune.PIDTuner;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
=======
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
>>>>>>> 5e13cd9fa02c26f9ef30f4835df78748d244b002
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
<<<<<<< HEAD
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
=======
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
>>>>>>> 5e13cd9fa02c26f9ef30f4835df78748d244b002
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.component.SwerveModule;
<<<<<<< HEAD
import frc.robot.util.MoPIDF;
import frc.robot.util.MoPrefs;
import frc.robot.util.MoShuffleboard;
import frc.robot.util.MutablePIDConstants;
import frc.robot.util.TunerUtils;
import java.util.function.Consumer;

public class DriveSubsystem extends SubsystemBase {
    /**
     * The maximum rate of turn that the drive will consider as equivalent to zero. Used to
     * determine when to re-enable heading pid after executing a driver-requested turn.
     */
    private static final double TURN_RATE_CUTOFF = 0.001;

    /** The maximum rate of movement that the drive will consider as equivalent to zero. */
    private static final double MOVE_RATE_CUTOFF = 0.05;

    private static final double RESET_ENCODER_INTERVAL = 0.5;
=======
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.utils.MutablePIDConstants;
import frc.robot.utils.TunerUtils;
import java.util.Map;
import java.util.function.Consumer;

public class DriveSubsystem extends SubsystemBase {

    /**
     * How frequently the absolute encoder will be used to re-zero the relative encoders.
     */
    private static final Time RESET_ENCODER_PERIOD = Units.Seconds.of(0.5);
>>>>>>> 5e13cd9fa02c26f9ef30f4835df78748d244b002

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
<<<<<<< HEAD
    public static final Measure<Distance> SWERVE_WHEEL_OFFSET = Units.Inch.of(11.5);

    private static enum TurnState {
        TURNING,
        HOLD_HEADING
    };

    private TurnState turnState = TurnState.HOLD_HEADING;
    private Rotation2d maintainHeading;

    private final MoPIDF headingController = new MoPIDF();
    private final PIDTuner headingTuner = TunerUtils.forMoPIDF(headingController, "Drive Heading");

    private GenericSubscriber shouldHeadingPID = MoShuffleboard.getInstance()
            .settingsTab
            .add("Keep Heading", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    public final SwerveModule frontLeft;
    public final SwerveModule frontRight;
    public final SwerveModule rearLeft;
    public final SwerveModule rearRight;

    private final Timer resetEncoderTimer = new Timer();

    public final MutablePIDConstants translationPathController = new MutablePIDConstants();
    public final MutablePIDConstants rotationPathController = new MutablePIDConstants();

    private final PIDTuner translationPathTuner =
            TunerUtils.forPathPlanner(translationPathController, "Follow Path Pos");
    private final PIDTuner rotationPathTuner = TunerUtils.forPathPlanner(rotationPathController, "Follow Path Rot");

    public final SwerveDriveKinematics kinematics;

    private final AHRS gyro;

    public boolean doResetEncoders = true;

    private LinearFilter desiredOffsetFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public DriveSubsystem(AHRS gyro) {
        this.gyro = gyro;
        maintainHeading = getCurrHeading();

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        this.frontLeft = new SwerveModule(
                "FL",
                new CANSparkMax(Constants.TURN_LEFT_FRONT.address(), MotorType.kBrushless),
                new TalonFX(Constants.DRIVE_LEFT_FRONT.address()),
                MoPrefs.flZero,
                MoPrefs.flRotScale,
                MoPrefs.flDistScale);

        this.frontRight = new SwerveModule(
                "FR",
                new CANSparkMax(Constants.TURN_RIGHT_FRONT.address(), MotorType.kBrushless),
                new TalonFX(Constants.DRIVE_RIGHT_FRONT.address()),
                MoPrefs.frZero,
                MoPrefs.frRotScale,
                MoPrefs.frDistScale);

        this.rearLeft = new SwerveModule(
                "RL",
                new CANSparkMax(Constants.TURN_LEFT_REAR.address(), MotorType.kBrushless),
                new TalonFX(Constants.DRIVE_LEFT_REAR.address()),
                MoPrefs.rlZero,
                MoPrefs.rlRotScale,
                MoPrefs.rlDistScale);

        this.rearRight = new SwerveModule(
                "RR",
                new CANSparkMax(Constants.TURN_RIGHT_REAR.address(), MotorType.kBrushless),
                new TalonFX(Constants.DRIVE_RIGHT_REAR.address()),
                MoPrefs.rrZero,
                MoPrefs.rrRotScale,
                MoPrefs.rrDistScale);

        resetEncoderTimer.start();

        MoShuffleboard.getInstance()
                .matchTab
                .addDouble(
                        "FL_POS", () -> frontLeft.driveMotor.getRotorPosition().getValueAsDouble());
        MoShuffleboard.getInstance()
                .matchTab
                .addDouble("FL_POS_m", () -> frontLeft.distEncoder.getPosition().in(Units.Meters));

=======
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
                MoPrefs.swerveFLZero,
                MoPrefs.swerveRotScale,
                MoPrefs.swerveDistScale);

        this.frontRight = new SwerveModule(
                "SWRV FR",
                new SparkMax(Constants.SWERVE_TURN_RIGHT_FRONT.address(), MotorType.kBrushless),
                new TalonFX(Constants.SWERVE_DRIVE_RIGHT_FRONT.address()),
                MoPrefs.swerveFRZero,
                MoPrefs.swerveRotScale,
                MoPrefs.swerveDistScale);

        this.rearLeft = new SwerveModule(
                "SWRV RL",
                new SparkMax(Constants.SWERVE_TURN_LEFT_REAR.address(), MotorType.kBrushless),
                new TalonFX(Constants.SWERVE_DRIVE_LEFT_REAR.address()),
                MoPrefs.swerveRLZero,
                MoPrefs.swerveRotScale,
                MoPrefs.swerveDistScale);

        this.rearRight = new SwerveModule(
                "SWRV RR",
                new SparkMax(Constants.SWERVE_TURN_RIGHT_REAR.address(), MotorType.kBrushless),
                new TalonFX(Constants.SWERVE_DRIVE_RIGHT_REAR.address()),
                MoPrefs.swerveRRZero,
                MoPrefs.swerveRotScale,
                MoPrefs.swerveDistScale);

        resetEncoderTimer.start();

>>>>>>> 5e13cd9fa02c26f9ef30f4835df78748d244b002
        double offset_meters = SWERVE_WHEEL_OFFSET.in(Units.Meters);
        Translation2d fl = new Translation2d(offset_meters, offset_meters);
        Translation2d fr = new Translation2d(offset_meters, -offset_meters);
        Translation2d rl = new Translation2d(-offset_meters, offset_meters);
        Translation2d rr = new Translation2d(-offset_meters, -offset_meters);

        this.kinematics = new SwerveDriveKinematics(fl, fr, rl, rr);
<<<<<<< HEAD
=======

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
>>>>>>> 5e13cd9fa02c26f9ef30f4835df78748d244b002
    }

    public SwerveModulePosition[] getWheelPositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()
        };
    }

<<<<<<< HEAD
    /**
     * Gets the current heading, within the range (-PI, PI]
     *
     * @return the current heading
     */
    private Rotation2d getCurrHeading() {
        return Rotation2d.fromRadians(MathUtil.angleModulus(gyro.getRotation2d().getRadians()));
    }

    /**
     * Calculate how much the robot should turn. We want to use PID to prevent any turning, except
     * for these situations: (1) if the driver has requested a turn, or (2) if the robot is slowing
     * down after a requested turn (to prevent an unexpected 'snap-back' behavior).
     *
     * @param turnRequest The turn requested by the driver
     * @param currentHeading The robot's current heading, as reported by the gyro
     * @return How much the robot should turn
     */
    private double calculateTurn(double turnRequest, Rotation2d currentHeading) {
        switch (turnState) {
            case HOLD_HEADING:
                if (turnRequest != 0) {
                    turnState = TurnState.TURNING;
                }
                break;
            case TURNING:
                if (turnRequest == 0 && Math.abs(gyro.getRate()) < TURN_RATE_CUTOFF) {
                    maintainHeading = currentHeading;
                    turnState = TurnState.HOLD_HEADING;
                }
                break;
        }
        switch (turnState) {
            case HOLD_HEADING:
                if (shouldHeadingPID.getBoolean(true)) {
                    return headingController.calculate(currentHeading.getRadians(), maintainHeading.getRadians());
                }
            case TURNING:
            default:
                return turnRequest;
        }
    }

=======
    private void forEachSwerveModule(Consumer<SwerveModule> forBody) {
        forBody.accept(frontLeft);
        forBody.accept(frontRight);
        forBody.accept(rearLeft);
        forBody.accept(rearRight);
    }

    /**
     * Robot-oriented cartesian drive
     */
>>>>>>> 5e13cd9fa02c26f9ef30f4835df78748d244b002
    public void driveCartesian(double fwdRequest, double leftRequest, double turnRequest) {
        this.driveCartesian(fwdRequest, leftRequest, turnRequest, new Rotation2d());
    }

<<<<<<< HEAD
    public void driveCartesian(
            double fwdRequest, double leftRequest, double turnRequest, Rotation2d fieldOrientedDriveAngle) {
        turnRequest = calculateTurn(turnRequest, getCurrHeading());

        var maxLinearSpeed = MoPrefs.maxDriveSpeed.get();
        var maxAngularSpeed = MoPrefs.maxTurnSpeed.get();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                maxLinearSpeed.times(fwdRequest).in(Units.MetersPerSecond),
                maxLinearSpeed.times(leftRequest).in(Units.MetersPerSecond),
                maxAngularSpeed.times(turnRequest).in(Units.RadiansPerSecond),
                fieldOrientedDriveAngle);
        driveRobotRelativeSpeeds(speeds);
    }

    public void driveCartesianPointAt(
            double fwdRequest, double leftRequest, Rotation2d fieldOrientedDriveAngle, Rotation2d desiredOffset) {

        Rotation2d offset = Rotation2d.fromRotations(desiredOffsetFilter.calculate(desiredOffset.getRotations()));

        Rotation2d currentHeading = getCurrHeading();
        Rotation2d desiredHeading = currentHeading.plus(offset);

        double turnRequest = headingController.calculate(currentHeading.getRadians(), desiredHeading.getRadians());

        driveCartesian(fwdRequest, leftRequest, turnRequest, fieldOrientedDriveAngle);
    }

    public void driveRobotRelativeSpeeds(ChassisSpeeds speeds) {
        driveSwerveStates(kinematics.toSwerveModuleStates(speeds));
    }

=======
    private MutLinearVelocity mutFwdRequest = Units.MetersPerSecond.mutable(0);
    private MutLinearVelocity mutLeftRequest = Units.MetersPerSecond.mutable(0);
    private MutAngularVelocity mutTurnRequest = Units.RotationsPerSecond.mutable(0);

    /**
     * Field-oriented cartesian drive
     */
    public void driveCartesian(
            double fwdRequest, double leftRequest, double turnRequest, Rotation2d fieldOrientedDriveAngle) {
        var maxLinearSpeed = MoPrefs.swerveMaxLinearSpeed.get();
        var maxAngularSpeed = MoPrefs.swerveMaxAngularSpeed.get();

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

>>>>>>> 5e13cd9fa02c26f9ef30f4835df78748d244b002
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(
                frontLeft.getState(), frontRight.getState(),
                rearLeft.getState(), rearRight.getState());
    }

<<<<<<< HEAD
    public void stop() {
        frontLeft.driveMotor.stopMotor();
        frontRight.driveMotor.stopMotor();
        rearLeft.driveMotor.stopMotor();
        rearRight.driveMotor.stopMotor();
    }

    private void driveSwerveStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MoPrefs.maxDriveSpeed.get());
=======
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
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MoPrefs.swerveMaxLinearSpeed.get());
>>>>>>> 5e13cd9fa02c26f9ef30f4835df78748d244b002

        frontLeft.drive(states[0]);
        frontRight.drive(states[1]);
        rearLeft.drive(states[2]);
        rearRight.drive(states[3]);
    }

<<<<<<< HEAD
    /**
     * Turn the robot in-place by the desired rotation.
     */
    public void rotateRelative(Rotation2d desiredRotation) {
        Rotation2d currentHeading = getCurrHeading();
        Rotation2d desiredHeading = currentHeading.plus(desiredRotation);

        double turnRequest = headingController.calculate(currentHeading.getRadians(), desiredHeading.getRadians());
        var angularSpeedRequest = MoPrefs.maxTurnSpeed.get().times(MathUtil.clamp(turnRequest, -1, 1));

        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, angularSpeedRequest.in(Units.RadiansPerSecond));
        driveRobotRelativeSpeeds(speeds);
    }

    public boolean isMoving() {
        return Math.abs(frontLeft.distEncoder.getVelocity().in(Units.MetersPerSecond)) > MOVE_RATE_CUTOFF
                || Math.abs(frontRight.distEncoder.getVelocity().in(Units.MetersPerSecond)) > MOVE_RATE_CUTOFF
                || Math.abs(rearLeft.distEncoder.getVelocity().in(Units.MetersPerSecond)) > MOVE_RATE_CUTOFF
                || Math.abs(rearRight.distEncoder.getVelocity().in(Units.MetersPerSecond)) > MOVE_RATE_CUTOFF;
    }

    public void forEachSwerveModule(Consumer<SwerveModule> forBody) {
        forBody.accept(frontLeft);
        forBody.accept(frontRight);
        forBody.accept(rearLeft);
        forBody.accept(rearRight);
    }

    public void resetRelativeEncoders() {
        if (!doResetEncoders) return;
=======
    public void resetRelativeEncoders() {
>>>>>>> 5e13cd9fa02c26f9ef30f4835df78748d244b002
        frontLeft.setRelativePosition();
        frontRight.setRelativePosition();
        rearLeft.setRelativePosition();
        rearRight.setRelativePosition();
    }

    @Override
    public void periodic() {
<<<<<<< HEAD
        if (resetEncoderTimer.advanceIfElapsed(RESET_ENCODER_INTERVAL)) {
            resetRelativeEncoders();
        }

        if (DriverStation.isDisabled()) {
            this.stop();
        }
=======
        if (resetEncoderTimer.advanceIfElapsed(RESET_ENCODER_PERIOD.in(Units.Seconds))) {
            resetRelativeEncoders();
        }
>>>>>>> 5e13cd9fa02c26f9ef30f4835df78748d244b002
    }
}
