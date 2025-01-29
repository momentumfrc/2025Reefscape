package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.encoder.MoRotationEncoder;
// import frc.robot.molib.pid.MoSparkMaxElevatorPID;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.utils.MoUtils;

public class ElevatorSubsystem extends SubsystemBase {
    private static final int ELEVATOR_CURRENT_LIMIT = 50;
    private static final int WRIST_CURRENT_LIMIT = 50;

    private final SparkMax elevatorA;
    private final SparkMax elevatorB;
    private final SparkMax elevatorWrist;
    private final VictorSPX endEffector;

    private SparkMaxConfig elevatorAConfig = new SparkMaxConfig();
    private SparkMaxConfig elevatorBConfig = new SparkMaxConfig();
    private SparkMaxConfig wristConfig = new SparkMaxConfig();
    private SoftLimitConfig elevatorLimitConfig = new SoftLimitConfig();
    private SoftLimitConfig wristLimitConfig = new SoftLimitConfig();

    public final MoRotationEncoder elevatorAbsEncoder;
    public final MoRotationEncoder wristAbsEncoder;
    public final MoRotationEncoder elevatorRelEncoder;
    public final MoRotationEncoder wristRelEncoder;

    // TODO:
    // private final MoSparkMaxElevatorPID elevatorVelocityPid;
    // private final MoSparkMaxElevatorPID wristVelocityPid;
    // private final MoSparkMaxElevatorPID elevatorSmartMotionPid;
    // private final MoSparkMaxElevatorPID wristSmartMotionPid;

    public static record ElevatorPosition(Angle elevatorAngle, Angle wristAngle) {}

    public static record ElevatorMovementRequest(double elevatorPower, double wristPower) {
        public ElevatorMovementRequest(double elevatorPower, double wristPower) {
            this.elevatorPower = MoUtils.clamp(elevatorPower, -1, 1);
            this.wristPower = MoUtils.clamp(wristPower, -1, 1);
        }

        public boolean isZero() {
            return Math.abs(elevatorPower) < Constants.FLOAT_EPSILON && Math.abs(wristPower) < Constants.FLOAT_EPSILON;
        }
    }

    public void configureMotors() {
        elevatorLimitConfig.reverseSoftLimit(0).reverseSoftLimitEnabled(true);
        elevatorAConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ELEVATOR_CURRENT_LIMIT)
                .apply(elevatorLimitConfig);
        elevatorBConfig.apply(elevatorAConfig).follow(elevatorA, true);
        wristConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(WRIST_CURRENT_LIMIT)
                .apply(wristLimitConfig);

        this.elevatorA.configure(elevatorAConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.elevatorB.configure(elevatorBConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.elevatorWrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        MoUtils.setupRelativeEncoder(
                elevatorRelEncoder,
                elevatorAbsEncoder.getPosition(),
                MoPrefs.elevatorAbsZero.get(),
                MoPrefs.elevatorEncoderScale.get());

        MoUtils.setupRelativeEncoder(
                wristRelEncoder,
                wristAbsEncoder.getPosition(),
                MoPrefs.wristAbsZero.get(),
                MoPrefs.wristEncoderScale.get());

        elevatorAbsEncoder.setConversionFactor(MoPrefs.elevatorAbsEncoderScale.get());
    }

    public ElevatorSubsystem() {
        super("Elevator");

        this.elevatorA = new SparkMax(Constants.ELEVATORA.address(), MotorType.kBrushless);
        this.elevatorB = new SparkMax(Constants.ELEVATORB.address(), MotorType.kBrushless);
        this.elevatorWrist = new SparkMax(Constants.ELEVATOR_WRIST.address(), MotorType.kBrushless);
        this.endEffector = new VictorSPX(Constants.END_EFFECTOR.address());

        elevatorAbsEncoder = MoRotationEncoder.forSparkAbsolute(elevatorA, Units.Rotations);
        wristAbsEncoder = MoRotationEncoder.forSparkAbsolute(elevatorWrist, Units.Rotations);

        elevatorRelEncoder = MoRotationEncoder.forSparkRelative(elevatorA, Units.Rotations);
        wristRelEncoder = MoRotationEncoder.forSparkRelative(elevatorWrist, Units.Rotations);

        // Setup listeners for encoder scales and absolute zeros. Use notifyImmediately on zero listeners to set the
        // values now.  TODO: ask Jordan what this means
        MoPrefs.elevatorEncoderScale.subscribe(scale -> MoUtils.setupRelativeEncoder(
                elevatorRelEncoder, elevatorAbsEncoder.getPosition(), MoPrefs.elevatorAbsZero.get(), scale));
        MoPrefs.elevatorAbsEncoderScale.subscribe(elevatorAbsEncoder::setConversionFactor, true);
        MoPrefs.wristEncoderScale.subscribe(scale -> MoUtils.setupRelativeEncoder(
                wristRelEncoder, wristAbsEncoder.getPosition(), MoPrefs.wristAbsZero.get(), scale));
        MoPrefs.elevatorAbsZero.subscribe(zero -> MoUtils.setupRelativeEncoder(
                elevatorRelEncoder, elevatorAbsEncoder.getPosition(), zero, MoPrefs.elevatorEncoderScale.get()));
        MoPrefs.wristAbsZero.subscribe(zero -> MoUtils.setupRelativeEncoder(
                wristRelEncoder, wristAbsEncoder.getPosition(), zero, MoPrefs.wristEncoderScale.get()));

        MoPrefs.elevatorMaxExtension.subscribe(limit ->
                elevatorLimitConfig.forwardSoftLimit((float) limit.in(elevatorRelEncoder.getInternalEncoderUnits())));
        MoPrefs.wristMaxExtension.subscribe(limit ->
                wristLimitConfig.forwardSoftLimit((float) limit.in(wristRelEncoder.getInternalEncoderUnits())));
    }

    public void intakeAlgaeCoralExtake() {
        endEffector.set(
                ControlMode.PercentOutput, MoPrefs.endEffectorPower.get().in(Units.Volts));
    }

    public void extakeAlgaeCoralIntake() {
        endEffector.set(
                ControlMode.PercentOutput, -MoPrefs.endEffectorPower.get().in(Units.Volts));
    }
}
