package frc.robot.subsystem;

import com.momentum4999.motune.PIDTuner;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
import java.util.function.Consumer;

public class ElevatorSubsystem extends SubsystemBase {
    private static final int ELEVATOR_CURRENT_LIMIT = 50;
    private static final int WRIST_CURRENT_LIMIT = 50;

    private MutCurrent wristCurrent = Units.Amps.mutable(0);
    private MutCurrent elevatorCurrent = Units.Amps.mutable(0);
    private MutAngle wristAngleFromHorizontal = Units.Rotations.mutable(0);

    public static enum ElevatorControlMode {
        SMARTMOTION,
        DIRECT_VELOCITY,
        FALLBACK_DIRECT_POWER
    };

    private final SparkMax elevatorA;
    private final SparkMax elevatorB;
    private final SparkMax elevatorWrist;

    private final SparkMaxConfig elevatorAConfig;
    private final SparkMaxConfig elevatorBConfig;
    private final SparkMaxConfig wristConfig;

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

    public final SendableChooser<ElevatorControlMode> controlMode;

    private final GenericEntry hasZero = MoShuffleboard.getInstance()
            .elevatorTab
            .add("Elevator Zeroed", false)
            .getEntry();

    private final GenericEntry wristLimitsEnabled = MoShuffleboard.getInstance()
            .elevatorTab
            .add("Wrist Limit Enabled", true)
            .getEntry();

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

    private void updateElevatorConfig(Consumer<SparkMaxConfig> configConsumer) {
        configConsumer.accept(elevatorAConfig);
        elevatorA.configure(elevatorAConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void updateWristConfig(Consumer<SparkMaxConfig> configConsumer) {
        configConsumer.accept(wristConfig);
        elevatorWrist.configure(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public ElevatorSubsystem() {
        super("Elevator");

        this.elevatorA = new SparkMax(Constants.ELEVATORA.address(), MotorType.kBrushless);
        this.elevatorB = new SparkMax(Constants.ELEVATORB.address(), MotorType.kBrushless);
        this.elevatorWrist = new SparkMax(Constants.ELEVATOR_WRIST.address(), MotorType.kBrushless);

        wristAbsEncoder = MoRotationEncoder.forSparkAbsolute(elevatorWrist, Units.Rotations);

        elevatorRelEncoder = MoDistanceEncoder.forSparkRelative(elevatorA, Units.Centimeters);
        wristRelEncoder = MoRotationEncoder.forSparkRelative(elevatorWrist, Units.Rotations);

        elevatorAConfig = new SparkMaxConfig();
        elevatorAConfig
                .softLimit
                .reverseSoftLimit(0)
                .reverseSoftLimitEnabled(false)
                .forwardSoftLimit(MoPrefs.elevatorMaxExtension.get().in(elevatorRelEncoder.getInternalEncoderUnits()))
                .forwardSoftLimitEnabled(false);
        elevatorAConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ELEVATOR_CURRENT_LIMIT);

        elevatorBConfig = new SparkMaxConfig();
        elevatorBConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ELEVATOR_CURRENT_LIMIT)
                .follow(elevatorA, true);

        wristConfig = new SparkMaxConfig();
        wristConfig
                .softLimit
                .reverseSoftLimit(MoPrefs.wristMinExtension.get().in(wristRelEncoder.getInternalEncoderUnits()))
                .reverseSoftLimitEnabled(true)
                .forwardSoftLimit(MoPrefs.wristMaxExtension.get().in(wristRelEncoder.getInternalEncoderUnits()))
                .forwardSoftLimitEnabled(true);
        wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(WRIST_CURRENT_LIMIT);

        this.elevatorA.configure(elevatorAConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.elevatorB.configure(elevatorBConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.elevatorWrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        reZeroWrist();

        // Setup listeners for config values
        MoPrefs.elevatorMaxExtension.subscribe(value -> updateElevatorConfig(
                config -> config.softLimit.forwardSoftLimit(value.in(elevatorRelEncoder.getInternalEncoderUnits()))));
        MoPrefs.wristMinExtension.subscribe(value -> updateWristConfig(
                config -> config.softLimit.reverseSoftLimit(value.in(wristRelEncoder.getInternalEncoderUnits()))));
        MoPrefs.wristMaxExtension.subscribe(value -> updateWristConfig(
                config -> config.softLimit.forwardSoftLimit(value.in(wristRelEncoder.getInternalEncoderUnits()))));

        // TODO: Right now encoder scales are configured separately through the MoEncoder framework. It'd be nice
        // if there was some way to configure the MoEncoder to update the persistent elevatorAConfig instead of creating
        // its own config.
        MoPrefs.elevatorEncoderScale.subscribe(value -> elevatorRelEncoder.setConversionFactor(value), true);

        MoPrefs.wristAbsZero.subscribe(value -> MoUtils.setupRelativeEncoder(
                wristRelEncoder, wristAbsEncoder.getPosition(), value, MoPrefs.wristEncoderScale.get()));
        MoPrefs.wristEncoderScale.subscribe(value -> MoUtils.setupRelativeEncoder(
                wristRelEncoder, wristAbsEncoder.getPosition(), MoPrefs.wristAbsZero.get(), value));

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

        // TODO: Right now PID constants are configured separately through the PIDTuner framework. It'd be nice
        // if there was some way to configure the PIDTuner to update the persistent elevatorAConfig instead of
        // creating its own config.
        elevatorVelTuner = TunerUtils.forMoSparkElevator(elevatorVelocityPid, "Elevator Vel.");
        wristVelTuner = TunerUtils.forMoSparkArm(wristVelocityPid, "Wrist Vel.");
        elevatorPosTuner = TunerUtils.forMoSparkElevator(elevatorSmartMotionPid, "Elevator Pos.");
        wristPosTuner = TunerUtils.forMoSparkArm(wristSmartMotionPid, "Wrist Pos.");

        MoShuffleboard.getInstance().elevatorTab.addDouble("Elevator Height (cm)", () -> getElevatorHeight()
                .in(Units.Centimeters));
        MoShuffleboard.getInstance().elevatorTab.addDouble("Wrist Rel. Angle (R)", () -> getWristAngle()
                .in(Units.Rotations));
        MoShuffleboard.getInstance().elevatorTab.addDouble("Wrist Abs. Angle (R)", () -> getWristAbsAngle()
                .in(Units.Rotations));

        controlMode = MoShuffleboard.enumToChooser(ElevatorControlMode.class);
        MoShuffleboard.getInstance().settingsTab.add("Elevator Control Mode", controlMode);

        MoShuffleboard.getInstance().elevatorTab.add(this);
    }

    public void reZeroWrist() {
        MoUtils.setupRelativeEncoder(
                wristRelEncoder,
                wristAbsEncoder.getPosition(),
                MoPrefs.wristAbsZero.get(),
                MoPrefs.wristEncoderScale.get());
    }

    public void disableWristLimit() {
        wristLimitsEnabled.setBoolean(false);
        updateWristConfig(config -> config.softLimit.reverseSoftLimitEnabled(false));
    }

    public void enableWristLimit() {
        wristLimitsEnabled.setBoolean(true);
        updateWristConfig(config -> config.softLimit.reverseSoftLimitEnabled(true));
    }

    public Distance getElevatorHeight() {
        return elevatorRelEncoder.getPosition();
    }

    public Angle getWristAngle() {
        return wristRelEncoder.getPosition();
    }

    public Angle getWristAbsAngle() {
        return wristAbsEncoder.getPosition();
    }

    private Angle getWristAngleFromHorizontal() {
        return wristAngleFromHorizontal
                .mut_replace(wristRelEncoder.getPosition())
                .mut_minus(MoPrefs.wristHorizontal.get());
    }

    private ElevatorMovementRequest limitElevatorMovementRequest(ElevatorMovementRequest request) {
        double elevatorPower = request.elevatorPower;
        double wristPower = request.wristPower;

        var elevatorPos = elevatorRelEncoder.getPosition();
        var wristPos = wristRelEncoder.getPosition();

        if (hasZero.getBoolean(false)) {
            if (elevatorPower > 0 && elevatorPos.gt(MoPrefs.elevatorMaxExtension.get())) {
                elevatorPower = 0;
            }
            if (elevatorPower < 0 && elevatorPos.lt(Units.Centimeters.zero())) {
                elevatorPower = 0;
            }
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

    public void moveForElevatorZeroing() {
        elevatorA.setVoltage(-1 * MoPrefs.elevatorZeroPower.get().in(Units.Volts));

        switch (controlMode.getSelected()) {
            case SMARTMOTION:
                var stowPos = ElevatorSetpointManager.getInstance().getSetpoint(ElevatorSetpoint.STOW);
                wristSmartMotionPid.setPositionReference(stowPos.wristAngle());
                break;
            case DIRECT_VELOCITY:
                wristVelocityPid.setVelocityReference(Units.RotationsPerSecond.zero());
                break;
            case FALLBACK_DIRECT_POWER:
                elevatorWrist.setVoltage(0);
                break;
        }
    }

    public void zeroElevator() {
        hasZero.setBoolean(true);
        elevatorRelEncoder.setPosition(Units.Centimeters.zero());
        updateElevatorConfig(
                config -> config.softLimit.reverseSoftLimitEnabled(true).forwardSoftLimitEnabled(true));
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

    public void adjustSmartPosition(ElevatorPosition position) {
        elevatorSmartMotionPid.setPositionReference(position.elevatorDistance);
        wristSmartMotionPid.setPositionReference(position.wristAngle);
    }

    public boolean atPosition(ElevatorPosition position) {
        double thresh = MoPrefs.elevatorSetpointVarianceThreshold.get().in(Units.Value);
        return elevatorRelEncoder.getPosition().isNear(position.elevatorDistance(), thresh)
                && wristRelEncoder.getPosition().isNear(position.wristAngle(), thresh);
    }

    public Current getWristCurrent() {
        return wristCurrent.mut_replace(elevatorWrist.getOutputCurrent(), Units.Amps);
    }

    public Current getElevatorCurrent() {
        return elevatorCurrent.mut_replace(elevatorA.getOutputCurrent(), Units.Amps);
    }

    public boolean hasZero() {
        return this.hasZero.getBoolean(false);
    }

    @Override
    public void periodic() {
        var elevatorAWarnings = elevatorA.getStickyWarnings();
        if (elevatorAWarnings.brownout || elevatorAWarnings.hasReset) {
            elevatorA.clearFaults();
            DriverStation.reportWarning("Elevator A Brownout", false);
            hasZero.setBoolean(false);

            elevatorAConfig.softLimit.reverseSoftLimitEnabled(false).forwardSoftLimitEnabled(false);
            elevatorA.configure(elevatorAConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

            elevatorRelEncoder.setConversionFactor(MoPrefs.elevatorEncoderScale.get());

            elevatorVelTuner.populatePIDValues();
            elevatorPosTuner.populatePIDValues();
        }

        var elevatorBWarnings = elevatorB.getStickyWarnings();
        if (elevatorBWarnings.brownout || elevatorBWarnings.hasReset) {
            elevatorB.clearFaults();
            DriverStation.reportWarning("Elevator B Brownout", false);

            elevatorB.configure(elevatorBConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        var wristWarnings = elevatorWrist.getStickyWarnings();
        if (wristWarnings.brownout || wristWarnings.hasReset) {
            elevatorWrist.clearFaults();
            DriverStation.reportWarning("Elevator Wrist Brownout", false);

            elevatorWrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            reZeroWrist();

            wristVelTuner.populatePIDValues();
            wristPosTuner.populatePIDValues();
        }
    }
}
