package frc.robot.subsystem;

import com.momentum4999.motune.PIDTuner;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import java.util.EnumSet;
import java.util.function.Consumer;

public class ElevatorSubsystem extends SubsystemBase {
    private static final int ELEVATOR_CURRENT_LIMIT = 50;
    private static final int WRIST_CURRENT_LIMIT = 50;

    private MutCurrent wristCurrent = Units.Amps.mutable(0);
    private MutCurrent elevatorCurrent = Units.Amps.mutable(0);
    private MutAngle wristAngleFromHorizontal = Units.Rotations.mutable(0);

    public static enum ElevatorControlMode {
        SMARTMOTION,
        RAW_POSITION_PID,
        DIRECT_VELOCITY,
        FALLBACK_DIRECT_POWER
    };

    private final SparkMax elevatorA;
    private final SparkMax elevatorB;
    private final SparkMax elevatorWrist;

    private final SparkMaxConfig elevatorAConfig = new SparkMaxConfig();
    private final SparkMaxConfig elevatorBConfig = new SparkMaxConfig();
    private final SparkMaxConfig wristConfig = new SparkMaxConfig();

    public final MoRotationEncoder wristAbsEncoder;
    public final MoDistanceEncoder elevatorRelEncoder;
    public final MoRotationEncoder wristRelEncoder;

    private final MoSparkMaxElevatorPID elevatorVelocityPid;
    private final MoSparkMaxArmPID wristVelocityPid;
    private final MoSparkMaxElevatorPID elevatorSmartMotionPid;
    private final MoSparkMaxArmPID wristSmartMotionPid;
    private final MoSparkMaxElevatorPID elevatorPositionPid;
    private final MoSparkMaxArmPID wristPositionPid;

    private final PIDTuner elevatorVelTuner;
    private final PIDTuner wristVelTuner;
    private final PIDTuner elevatorSmartPosTuner;
    private final PIDTuner wristSmartPosTuner;
    private final PIDTuner elevatorPosTuner;
    private final PIDTuner wristPosTuner;

    public final SendableChooser<ElevatorControlMode> controlMode;

    private final GenericEntry hasZero = MoShuffleboard.getInstance()
            .elevatorTab
            .add("Elevator Zeroed", false)
            .getEntry();

    private final GenericEntry nominalReverseLimitEnabled = MoShuffleboard.getInstance()
            .elevatorTab
            .add("Wrist Nominal Reverse Limit Enabled", true)
            .getEntry();

    public static record ElevatorPosition(Distance elevatorDistance, Angle wristAngle) {}

    public static record ElevatorMovementRequest(double elevatorPower, double wristPower) {
        public ElevatorMovementRequest(double elevatorPower, double wristPower) {
            this.elevatorPower = MathUtil.clamp(elevatorPower, -1, 1);
            this.wristPower = MathUtil.clamp(wristPower, -1, 1);
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

        wristAbsEncoder = MoRotationEncoder.forSparkAbsolute(elevatorWrist, Units.Rotations, () -> wristConfig);

        elevatorRelEncoder = MoDistanceEncoder.forSparkRelative(elevatorA, Units.Centimeters, () -> elevatorAConfig);
        wristRelEncoder = MoRotationEncoder.forSparkRelative(elevatorWrist, Units.Rotations, () -> wristConfig);

        elevatorAConfig
                .softLimit
                .reverseSoftLimit(0)
                .reverseSoftLimitEnabled(false)
                .forwardSoftLimit(MoPrefs.elevatorMaxExtension.get().in(elevatorRelEncoder.getInternalEncoderUnits()))
                .forwardSoftLimitEnabled(false);
        elevatorAConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ELEVATOR_CURRENT_LIMIT);

        elevatorBConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ELEVATOR_CURRENT_LIMIT)
                .follow(elevatorA, true);

        wristConfig
                .softLimit
                .reverseSoftLimit(MoPrefs.wristNominalRevLimit.get().in(wristRelEncoder.getInternalEncoderUnits()))
                .reverseSoftLimitEnabled(true)
                .forwardSoftLimit(MoPrefs.wristMaxExtension.get().in(wristRelEncoder.getInternalEncoderUnits()))
                .forwardSoftLimitEnabled(true);
        wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(WRIST_CURRENT_LIMIT);

        this.elevatorA.configure(elevatorAConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.elevatorB.configure(elevatorBConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.elevatorWrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // TODO: this syntax sucks. Make a nice MoTables wrapper
        var coastMotorsEntry = NetworkTableInstance.getDefault()
                .getTable("Shuffleboard/Elevator")
                .getEntry("Coast Motors");
        coastMotorsEntry.setBoolean(false);
        coastMotorsEntry
                .getInstance()
                .addListener(coastMotorsEntry, EnumSet.of(NetworkTableEvent.Kind.kValueAll), coast -> {
                    System.out.println(coast.valueData.value.getBoolean());
                    var idleMode = coast.valueData.value.getBoolean() ? IdleMode.kCoast : IdleMode.kBrake;
                    elevatorAConfig.idleMode(idleMode);
                    elevatorBConfig.idleMode(idleMode);
                    wristConfig.idleMode(idleMode);
                    elevatorA.configure(
                            elevatorAConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                    elevatorB.configure(
                            elevatorBConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                    elevatorWrist.configure(
                            wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                });

        reZeroWrist();

        // Setup listeners for config values
        MoPrefs.elevatorMaxExtension.subscribe(value -> updateElevatorConfig(
                config -> config.softLimit.forwardSoftLimit(value.in(elevatorRelEncoder.getInternalEncoderUnits()))));
        MoPrefs.wristNominalRevLimit.subscribe(value -> updateWristConfig(
                config -> config.softLimit.reverseSoftLimit(value.in(wristRelEncoder.getInternalEncoderUnits()))));
        MoPrefs.wristMaxExtension.subscribe(value -> updateWristConfig(
                config -> config.softLimit.forwardSoftLimit(value.in(wristRelEncoder.getInternalEncoderUnits()))));

        MoPrefs.elevatorEncoderScale.subscribe(value -> elevatorRelEncoder.setConversionFactor(value), true);

        MoPrefs.wristAbsZero.subscribe(value -> MoUtils.setupRelativeEncoder(
                wristRelEncoder, wristAbsEncoder.getPosition(), value, MoPrefs.wristEncoderScale.get()));
        MoPrefs.wristEncoderScale.subscribe(value -> MoUtils.setupRelativeEncoder(
                wristRelEncoder, wristAbsEncoder.getPosition(), MoPrefs.wristAbsZero.get(), value));

        elevatorVelocityPid = new MoSparkMaxElevatorPID(
                MoSparkMaxPID.Type.VELOCITY,
                elevatorA,
                ClosedLoopSlot.kSlot0,
                elevatorRelEncoder,
                () -> elevatorAConfig);
        wristVelocityPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.VELOCITY,
                elevatorWrist,
                ClosedLoopSlot.kSlot0,
                wristRelEncoder,
                this::getWristAngleFromHorizontal,
                () -> wristConfig);
        elevatorSmartMotionPid = new MoSparkMaxElevatorPID(
                MoSparkMaxPID.Type.SMARTMOTION,
                elevatorA,
                ClosedLoopSlot.kSlot1,
                elevatorRelEncoder,
                () -> elevatorAConfig);
        wristSmartMotionPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.SMARTMOTION,
                elevatorWrist,
                ClosedLoopSlot.kSlot1,
                wristRelEncoder,
                this::getWristAngleFromHorizontal,
                () -> wristConfig);
        elevatorPositionPid = new MoSparkMaxElevatorPID(
                MoSparkMaxPID.Type.POSITION,
                elevatorA,
                ClosedLoopSlot.kSlot2,
                elevatorRelEncoder,
                () -> elevatorAConfig);
        wristPositionPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.POSITION,
                elevatorWrist,
                ClosedLoopSlot.kSlot2,
                wristRelEncoder,
                this::getWristAngleFromHorizontal,
                () -> wristConfig);

        elevatorVelTuner = TunerUtils.forMoSparkElevator(elevatorVelocityPid, "Elevator Vel.");
        wristVelTuner = TunerUtils.forMoSparkArm(wristVelocityPid, "Wrist Vel.");
        elevatorSmartPosTuner = TunerUtils.forMoSparkElevator(elevatorSmartMotionPid, "Elevator Smart Pos.");
        wristSmartPosTuner = TunerUtils.forMoSparkArm(wristSmartMotionPid, "Wrist Smart Pos.");
        elevatorPosTuner = TunerUtils.forMoSparkElevator(elevatorPositionPid, "Elevator Pos.");
        wristPosTuner = TunerUtils.forMoSparkArm(wristPositionPid, "Wrist Pos.");

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

    public boolean isWristNominalReverseLimitEnabled() {
        return nominalReverseLimitEnabled.getBoolean(true);
    }

    public boolean isWristInDanger() {
        return wristRelEncoder.getPosition().lt(MoPrefs.wristNominalRevLimit.get());
    }

    public void disableWristNominalReverseLimit() {
        nominalReverseLimitEnabled.setBoolean(false);
        updateWristConfig(config -> config.softLimit.reverseSoftLimit(0));
    }

    public void enableWristNominalReverseLimit() {
        nominalReverseLimitEnabled.setBoolean(true);
        updateWristConfig(config -> config.softLimit.reverseSoftLimit(
                MoPrefs.wristNominalRevLimit.get().in(wristRelEncoder.getInternalEncoderUnits())));
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

    public void adjustPosition(ElevatorPosition position) {
        double varianceThreshold =
                MoPrefs.elevatorSetpointVarianceThreshold.get().in(Units.Value);
        boolean cutElevatorPower = position.elevatorDistance.isNear(Units.Centimeters.zero(), varianceThreshold)
                && getElevatorHeight().isNear(Units.Centimeters.zero(), varianceThreshold);

        MoSparkMaxElevatorPID currModeElevatorPid;
        MoSparkMaxArmPID currModeWristPid;

        if (controlMode.getSelected() == ElevatorControlMode.SMARTMOTION) {
            currModeElevatorPid = elevatorSmartMotionPid;
            currModeWristPid = wristSmartMotionPid;
        } else {
            currModeElevatorPid = elevatorPositionPid;
            currModeWristPid = wristPositionPid;
        }

        if (cutElevatorPower) {
            elevatorA.setVoltage(0);
        } else {
            currModeElevatorPid.setPositionReference(position.elevatorDistance);
        }
        currModeWristPid.setPositionReference(position.wristAngle);
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

    public SysIdRoutine.Mechanism getElevatorSysidMechanism() {
        final MutVoltage mut_volt = Units.Volts.mutable(0);

        return new SysIdRoutine.Mechanism(
                v -> {
                    elevatorA.setVoltage(v.in(Units.Volts));
                    elevatorWrist.stopMotor();
                },
                log -> {
                    log.motor("elevatorAMtr")
                            .voltage(mut_volt.mut_replace(
                                    elevatorA.getAppliedOutput() * elevatorA.getBusVoltage(), Units.Volts))
                            .linearPosition(elevatorRelEncoder.getPosition())
                            .linearVelocity(elevatorRelEncoder.getVelocity());
                },
                this);
    }

    public SysIdRoutine.Mechanism getWristSysidMechanism() {
        final MutVoltage mut_volt = Units.Volts.mutable(0);

        return new SysIdRoutine.Mechanism(
                v -> {
                    elevatorA.stopMotor();
                    elevatorWrist.setVoltage(v);
                },
                log -> {
                    log.motor("wristMtr")
                            .voltage(mut_volt.mut_replace(
                                    elevatorWrist.getAppliedOutput() * elevatorWrist.getBusVoltage(), Units.Volts))
                            .angularPosition(wristRelEncoder.getPosition())
                            .angularVelocity(wristRelEncoder.getVelocity());
                },
                this);
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
        }
    }
}
