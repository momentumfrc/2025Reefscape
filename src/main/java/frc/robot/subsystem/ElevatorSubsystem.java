package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.momentum4999.motune.PIDTuner;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.encoder.MoDistanceEncoder;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.pid.MoSparkMaxArmPID;
import frc.robot.molib.pid.MoSparkMaxElevatorPID;
import frc.robot.molib.pid.MoSparkMaxPID;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.utils.MoUtils;
import frc.robot.utils.TunerUtils;

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
    public final MoDistanceEncoder elevatorRelEncoder;
    public final MoRotationEncoder wristRelEncoder;

    private final MoSparkMaxElevatorPID elevatorVelocityPid;
    private final MoSparkMaxArmPID wristVelocityPid;
    private final MoSparkMaxElevatorPID elevatorSmartMotionPid;
    private final MoSparkMaxArmPID wristSmartMotionPid;

    private final PIDTuner elevatorVelTuner;
    private final PIDTuner wristVelTuner;
    private final PIDTuner elevatorPosTuner;
    private final PIDTuner wristPosTuner;

    public static record ElevatorPosition(Distance elevatorDistance, Angle wristAngle) {}

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

        elevatorVelTuner.populatePIDValues();
        wristVelTuner.populatePIDValues();
        elevatorPosTuner.populatePIDValues();
        wristPosTuner.populatePIDValues();
    }

    public ElevatorSubsystem() {
        super("Elevator");

        this.elevatorA = new SparkMax(Constants.ELEVATORA.address(), MotorType.kBrushless);
        this.elevatorB = new SparkMax(Constants.ELEVATORB.address(), MotorType.kBrushless);
        this.elevatorWrist = new SparkMax(Constants.ELEVATOR_WRIST.address(), MotorType.kBrushless);
        this.endEffector = new VictorSPX(Constants.END_EFFECTOR.address());

        elevatorAbsEncoder = MoRotationEncoder.forSparkAbsolute(elevatorA, Units.Rotations);
        wristAbsEncoder = MoRotationEncoder.forSparkAbsolute(elevatorWrist, Units.Rotations);

        elevatorRelEncoder = MoDistanceEncoder.forSparkRelative(elevatorA, Units.Centimeters);
        wristRelEncoder = MoRotationEncoder.forSparkRelative(elevatorWrist, Units.Rotations);

        // Setup listeners for encoder scales and absolute zeros. Use notifyImmediately on zero listeners to set the
        // values now.
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

        elevatorVelocityPid = new MoSparkMaxElevatorPID(
                MoSparkMaxPID.Type.VELOCITY, elevatorA, ClosedLoopSlot.kSlot0, elevatorRelEncoder);
        wristVelocityPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.VELOCITY,
                elevatorWrist,
                ClosedLoopSlot.kSlot0,
                wristRelEncoder,
                this::getWristAngleFromHorizontal);
        elevatorSmartMotionPid = new MoSparkMaxElevatorPID(
                MoSparkMaxPID.Type.SMARTMOTION, elevatorA, ClosedLoopSlot.kSlot1, elevatorRelEncoder);
        wristSmartMotionPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.SMARTMOTION,
                elevatorWrist,
                ClosedLoopSlot.kSlot1,
                wristRelEncoder,
                this::getWristAngleFromHorizontal);

        elevatorVelTuner = TunerUtils.forMoSparkElevator(elevatorVelocityPid, "Elevator Vel.");
        wristVelTuner = TunerUtils.forMoSparkArm(wristVelocityPid, "Wrist Vel.");
        elevatorPosTuner = TunerUtils.forMoSparkElevator(elevatorSmartMotionPid, "Elevator Pos.");
        wristPosTuner = TunerUtils.forMoSparkArm(wristSmartMotionPid, "Wrist Pos.");
    }

    private Distance getElevatorDistanceFromBottom() {
        return elevatorRelEncoder.getPosition().minus(MoPrefs.elevatorBottom.get());
    }

    private Angle getWristAngleFromHorizontal() {
        return wristRelEncoder.getPosition().minus(MoPrefs.wristHorizontal.get());
    }

    private ElevatorMovementRequest limitElevatorMovementRequest(ElevatorMovementRequest request) {
        double elevatorPower = request.elevatorPower;
        double wristPower = request.wristPower;

        var elevatorPos = elevatorRelEncoder.getPosition();
        var wristPos = wristRelEncoder.getPosition();

        if (elevatorPower > 0 && elevatorPos.gt(MoPrefs.elevatorMaxExtension.get())) {
            elevatorPower = 0;
        }
        if (elevatorPower < 0 && elevatorPos.lt(Units.Centimeters.zero())) {
            elevatorPower = 0;
        }
        if (wristPower > 0 && wristPos.gt(MoPrefs.wristMaxExtension.get())) {
            wristPower = 0;
        }
        if (wristPower < 0 && wristPos.lt(Units.Rotations.zero())) {
            wristPower = 0;
        }

        return new ElevatorMovementRequest(elevatorPower, wristPower);
    }

    public void adjustVelocity(ElevatorMovementRequest request) {
        request = limitElevatorMovementRequest(request);

        Measure<LinearVelocityUnit> elevatorVelocity =
                MoPrefs.elevatorMaxRps.get().times(request.elevatorPower);
        Measure<AngularVelocityUnit> wristVelocity = MoPrefs.wristMaxRps.get().times(request.wristPower);

        elevatorVelocityPid.setVelocityReference(elevatorVelocity);
        wristVelocityPid.setVelocityReference(wristVelocity);
    }

    public void adjustSmartPosition(ElevatorPosition position) {
        elevatorSmartMotionPid.setPositionReference(position.elevatorDistance);
        wristSmartMotionPid.setPositionReference(position.wristAngle);
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
