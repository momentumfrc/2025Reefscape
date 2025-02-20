package frc.robot.subsystem;

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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.command.elevator.TiltBackElevatorWristCommand;
import frc.robot.component.ElevatorSetpointManager;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.molib.MoShuffleboard;
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

    private MutCurrent wristCurrent = Units.Amps.mutable(0);
    private MutCurrent elevatorCurrent = Units.Amps.mutable(0);

    public static enum ElevatorControlMode {
        SMARTMOTION,
        DIRECT_VELOCITY,
        FALLBACK_DIRECT_POWER
    };

    public static enum WristState {
        NORMAL,
        HOLDING,
        RETURNING
    }

    public WristState state = WristState.NORMAL;

    private final SparkMax elevatorA;
    private final SparkMax elevatorB;
    private final SparkMax elevatorWrist;

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

    private TiltBackElevatorWristCommand tiltback;

    public final SendableChooser<ElevatorControlMode> controlMode;

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
        wristLimitConfig.reverseSoftLimit(0).reverseSoftLimitEnabled(true);
        elevatorAConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ELEVATOR_CURRENT_LIMIT)
                .apply(elevatorLimitConfig);
        elevatorBConfig.apply(elevatorAConfig).follow(elevatorA, true);
        wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(WRIST_CURRENT_LIMIT);
        adjustWristReverseLimit();

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

        MoPrefs.elevatorMaxExtension.subscribe(limit -> configureMotors());
        MoPrefs.wristMaxExtension.subscribe(limit -> configureMotors());

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
        configureMotors();
        controlMode = MoShuffleboard.enumToChooser(ElevatorControlMode.class);
        MoShuffleboard.getInstance().settingsTab.add("Elevator Control Mode", controlMode);

        MoShuffleboard.getInstance().elevatorTab.add(this);

        tiltback = new TiltBackElevatorWristCommand(this);
    }

    public void reZeroElevator() {
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
    }

    public void adjustWristReverseLimit() {
        wristConfig.apply(wristLimitConfig);
    }

    public void disableWristReverseLimit() {
        wristLimitConfig.reverseSoftLimitEnabled(false);
        adjustWristReverseLimit();
    }

    public void enableWristReverseLimit() {
        wristLimitConfig.reverseSoftLimitEnabled(true);
        adjustWristReverseLimit();
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

    public ElevatorPosition getElevatorPosition() {
        return new ElevatorPosition(elevatorRelEncoder.getPosition(), wristRelEncoder.getPosition());
    }

    public void adjustDirectPower(ElevatorMovementRequest request) {
        request = limitElevatorMovementRequest(request);
        elevatorA.set(request.elevatorPower);
        elevatorWrist.set(request.wristPower);
    }

    public void adjustVelocity(ElevatorMovementRequest request) {
        request = limitElevatorMovementRequest(request);

        Measure<LinearVelocityUnit> elevatorVelocity =
                MoPrefs.elevatorMaxRps.get().times(request.elevatorPower);
        Measure<AngularVelocityUnit> wristVelocity = MoPrefs.wristMaxRps.get().times(request.wristPower);

        elevatorVelocityPid.setVelocityReference(elevatorVelocity);
        wristVelocityPid.setVelocityReference(wristVelocity);
    }

    public void adjustElevatorSmartPosition(ElevatorPosition position) {
        elevatorSmartMotionPid.setPositionReference(position.elevatorDistance);
    }

    public void adjustWristSmartPosition(ElevatorPosition position) {
        wristSmartMotionPid.setPositionReference(position.wristAngle);
    }

    public void adjustSmartPosition(ElevatorPosition position) {
        switch (state) {
            case NORMAL:
            default:
                if (position.equals(ElevatorSetpointManager.getInstance().getSetpoint(ElevatorSetpoint.INTAKE))
                        && atPosition(position)) {
                    disableWristReverseLimit();
                    new ScheduleCommand(tiltback);
                    state = WristState.HOLDING;
                } else if (!atHeight(position) && !stowedWrist(position)) stowWrist();
                else if (stowedWrist(position)) adjustElevatorSmartPosition(position);
                else adjustWristSmartPosition(position);
                break;
            case HOLDING:
                if (!tiltback.isScheduled()) holdWristIn();
                if (!atHeight(position)) {
                    if (tiltback.isScheduled()) tiltback.cancel();
                    state = WristState.RETURNING;
                }
                break;
            case RETURNING:
                stowWrist();
                if (stowedWrist(position)) {
                    enableWristReverseLimit();
                    state = WristState.NORMAL;
                }
                break;
        }
    }

    public void stowWrist() {
        wristSmartMotionPid.setPositionReference(
                ElevatorSetpointManager.getInstance().getSetpoint(ElevatorSetpoint.STOW).wristAngle);
    }

    public void tiltBack() {
        elevatorWrist.setVoltage(-MoPrefs.elevatorWristPower.get().in(Units.Volts));
    }

    public void holdWristIn() {
        elevatorWrist.setVoltage(MoPrefs.elevatorWristHoldPower.get().in(Units.Volts));
    }

    public boolean atPosition(ElevatorPosition position) {
        double thresh = MoPrefs.elevatorSetpointVarianceThreshold.get().in(Units.Value);
        return elevatorRelEncoder.getPosition().isNear(position.elevatorDistance(), thresh)
                && wristRelEncoder.getPosition().isNear(position.wristAngle(), thresh);
    }

    public boolean atHeight(ElevatorPosition position) {
        double thresh = MoPrefs.elevatorSetpointVarianceThreshold.get().in(Units.Value);
        return elevatorRelEncoder.getPosition().isNear(position.elevatorDistance(), thresh);
    }

    public boolean stowedWrist(ElevatorPosition position) {
        double thresh = MoPrefs.elevatorSetpointVarianceThreshold.get().in(Units.Value);
        return wristRelEncoder.getPosition().isNear(Units.Inches.of(0), thresh);
    }

    public Current getWristCurrent() {
        return wristCurrent.mut_replace(elevatorWrist.getOutputCurrent(), Units.Amps);
    }

    public Current getElevatorCurrent() {
        return elevatorCurrent.mut_replace(elevatorA.getOutputCurrent(), Units.Amps);
    }
}
