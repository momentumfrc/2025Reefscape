package frc.robot.subsystem;

import com.momentum4999.motune.PIDTuner;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.component.ElevatorSetpointManager;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.MoSparkConfigurator;
import frc.robot.molib.encoder.MoDistanceEncoder;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.pid.MoSparkMaxArmPID;
import frc.robot.molib.pid.MoSparkMaxElevatorPID;
import frc.robot.molib.pid.MoSparkMaxPID;
import frc.robot.molib.pid.MoTrapezoidArmController;
import frc.robot.molib.pid.MoTrapezoidElevatorController;
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
    private MutAngle wristAbsEncoderValue = Units.Rotations.mutable(0);

    public static enum ElevatorControlMode {
        SMARTMOTION,
        DIRECT_VELOCITY,
        RAW_POSITION_PID,
        FALLBACK_DIRECT_POWER
    };

    private final SparkMax elevatorA;
    private final SparkMax elevatorB;
    private final SparkMax elevatorWrist;

    private final MoSparkConfigurator elevatorAConfig;
    private final MoSparkConfigurator elevatorBConfig;
    private final MoSparkConfigurator elevatorWristConfig;

    public final MoRotationEncoder wristAbsEncoder;
    public final MoDistanceEncoder elevatorRelEncoder;
    public final MoRotationEncoder wristRelEncoder;

    private final MoSparkMaxElevatorPID elevatorVelocityPid;
    private final MoSparkMaxArmPID wristVelocityPid;
    private final MoSparkMaxElevatorPID elevatorPositionPid;
    private final MoSparkMaxArmPID wristPositionPid;
    private final MoTrapezoidElevatorController elevatorTrapPid;
    private final MoTrapezoidArmController wristTrapPid;

    private final PIDTuner elevatorVelTuner;
    private final PIDTuner wristVelTuner;
    private final PIDTuner elevatorPosTuner;
    private final PIDTuner wristPosTuner;
    private final PIDTuner elevatorTrapPosTuner;
    private final PIDTuner wristTrapPosTuner;

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

    public ElevatorSubsystem() {
        super("Elevator");

        this.elevatorA = new SparkMax(Constants.ELEVATORA.address(), MotorType.kBrushless);
        this.elevatorB = new SparkMax(Constants.ELEVATORB.address(), MotorType.kBrushless);
        this.elevatorWrist = new SparkMax(Constants.ELEVATOR_WRIST.address(), MotorType.kBrushless);

        this.elevatorAConfig = MoSparkConfigurator.forSparkMax(elevatorA);
        this.elevatorBConfig = MoSparkConfigurator.forSparkMax(elevatorB);
        this.elevatorWristConfig = MoSparkConfigurator.forSparkMax(elevatorWrist);

        wristAbsEncoder = MoRotationEncoder.forSparkAbsolute(
                elevatorWrist.getAbsoluteEncoder(), Units.Rotations, elevatorWristConfig);
        wristAbsEncoder.setInverted(true);

        elevatorRelEncoder =
                MoDistanceEncoder.forSparkRelative(elevatorA.getEncoder(), Units.Centimeters, elevatorAConfig);
        wristRelEncoder =
                MoRotationEncoder.forSparkRelative(elevatorWrist.getEncoder(), Units.Rotations, elevatorWristConfig);

        elevatorAConfig.accept(config -> {
            config.softLimit
                    .reverseSoftLimit(0)
                    .reverseSoftLimitEnabled(false)
                    .forwardSoftLimit(
                            MoPrefs.elevatorMaxExtension.get().in(elevatorRelEncoder.getInternalEncoderUnits()))
                    .forwardSoftLimitEnabled(false);
            config.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(ELEVATOR_CURRENT_LIMIT);
        });

        elevatorBConfig.accept(config -> config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ELEVATOR_CURRENT_LIMIT)
                .follow(elevatorA, true));

        elevatorWristConfig.accept(config -> {
            config.softLimit
                    .reverseSoftLimit(MoPrefs.wristNominalRevLimit.get().in(wristRelEncoder.getInternalEncoderUnits()))
                    .reverseSoftLimitEnabled(true)
                    .forwardSoftLimit(MoPrefs.wristMaxExtension.get().in(wristRelEncoder.getInternalEncoderUnits()))
                    .forwardSoftLimitEnabled(true);
            config.closedLoopRampRate(MoPrefs.wristRampTime.get().in(Units.Seconds))
                    .inverted(true)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(WRIST_CURRENT_LIMIT);
        });

        // TODO: this syntax sucks. Make a nice MoTables wrapper
        var coastMotorsEntry = NetworkTableInstance.getDefault()
                .getTable("Shuffleboard/Elevator")
                .getEntry("Coast Motors");
        coastMotorsEntry.setBoolean(false);
        coastMotorsEntry
                .getInstance()
                .addListener(coastMotorsEntry, EnumSet.of(NetworkTableEvent.Kind.kValueAll), coast -> {
                    var idleMode = coast.valueData.value.getBoolean() ? IdleMode.kCoast : IdleMode.kBrake;
                    Consumer<SparkBaseConfig> configurator = config -> config.idleMode(idleMode);
                    elevatorAConfig.accept(configurator);
                    elevatorBConfig.accept(configurator);
                    elevatorWristConfig.accept(configurator);
                });

        // Setup listeners for config values
        MoPrefs.elevatorMaxExtension.subscribe(value -> elevatorAConfig.accept(
                config -> config.softLimit.forwardSoftLimit(value.in(elevatorRelEncoder.getInternalEncoderUnits()))));
        MoPrefs.wristNominalRevLimit.subscribe(value -> elevatorWristConfig.accept(
                config -> config.softLimit.reverseSoftLimit(value.in(wristRelEncoder.getInternalEncoderUnits()))));
        MoPrefs.wristMaxExtension.subscribe(value -> elevatorWristConfig.accept(
                config -> config.softLimit.forwardSoftLimit(value.in(wristRelEncoder.getInternalEncoderUnits()))));
        MoPrefs.wristRampTime.subscribe(
                value -> elevatorWristConfig.accept(config -> config.closedLoopRampRate(value.in(Units.Seconds))));

        MoPrefs.elevatorEncoderScale.subscribe(value -> elevatorRelEncoder.setConversionFactor(value), true);

        MoPrefs.wristAbsZero.subscribe(value -> MoUtils.setupRelativeEncoder(
                wristRelEncoder, wristAbsEncoder.getPosition(), value, MoPrefs.wristEncoderScale.get()));
        MoPrefs.wristEncoderScale.subscribe(value -> MoUtils.setupRelativeEncoder(
                wristRelEncoder, wristAbsEncoder.getPosition(), MoPrefs.wristAbsZero.get(), value));

        elevatorVelocityPid = new MoSparkMaxElevatorPID(
                MoSparkMaxPID.Type.VELOCITY, elevatorA, ClosedLoopSlot.kSlot0, elevatorRelEncoder, elevatorAConfig);
        wristVelocityPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.VELOCITY,
                elevatorWrist,
                ClosedLoopSlot.kSlot0,
                wristRelEncoder,
                this::getWristAngleFromHorizontal,
                elevatorWristConfig);
        elevatorPositionPid = new MoSparkMaxElevatorPID(
                MoSparkMaxPID.Type.POSITION, elevatorA, ClosedLoopSlot.kSlot1, elevatorRelEncoder, elevatorAConfig);
        wristPositionPid = new MoSparkMaxArmPID(
                MoSparkMaxPID.Type.POSITION,
                elevatorWrist,
                ClosedLoopSlot.kSlot1,
                wristRelEncoder,
                this::getWristAngleFromHorizontal,
                elevatorWristConfig);
        elevatorTrapPid = new MoTrapezoidElevatorController(
                "Elevator",
                elevatorA.getClosedLoopController(),
                ClosedLoopSlot.kSlot2,
                elevatorRelEncoder,
                elevatorAConfig);
        wristTrapPid = new MoTrapezoidArmController(
                "Wrist",
                elevatorWrist.getClosedLoopController(),
                ClosedLoopSlot.kSlot2,
                wristAbsEncoder,
                elevatorWristConfig,
                this::getWristAngleFromHorizontal);

        elevatorVelTuner = TunerUtils.forMoSparkElevator(elevatorVelocityPid, "Elevator Vel.");
        wristVelTuner = TunerUtils.forMoSparkArm(wristVelocityPid, "Wrist Vel.");
        elevatorPosTuner = TunerUtils.forMoSparkElevator(elevatorPositionPid, "Elevator Pos.");
        wristPosTuner = TunerUtils.forMoSparkArm(wristPositionPid, "Wrist Pos.");
        elevatorTrapPosTuner = TunerUtils.forMoTrapezoidElevator(elevatorTrapPid, "Elevator Trap Pos.");
        wristTrapPosTuner = TunerUtils.forMoTrapezoidArm(wristTrapPid, "Wrist Trap Pos.");

        MoShuffleboard.getInstance().elevatorTab.addDouble("Elevator Height (cm)", () -> getElevatorHeight()
                .in(Units.Centimeters));
        MoShuffleboard.getInstance().elevatorTab.addDouble("Wrist Rel. Angle (R)", () -> getWristAngle()
                .in(Units.Rotations));
        MoShuffleboard.getInstance().elevatorTab.addDouble("Wrist Abs. Angle (DEG)", () -> getWristAbsAngleZeroed()
                .in(Units.Degrees));
        MoShuffleboard.getInstance().elevatorTab.addDouble("Wrist Horiz Angle (R)", () -> getWristAngleFromHorizontal()
                .in(Units.Rotations));

        MoShuffleboard.getInstance().elevatorTab.addDouble("Elevator Current", () -> getElevatorCurrent()
                .in(Units.Amps));

        MoShuffleboard.getInstance().elevatorTab.addBoolean("Wrist In Danger?", this::isWristInDanger);

        controlMode = MoShuffleboard.enumToChooser(ElevatorControlMode.class);
        MoShuffleboard.getInstance().settingsTab.add("Elevator Control Mode", controlMode);

        MoShuffleboard.getInstance().elevatorTab.add(this);

        reZeroWrist();
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
        return getWristAbsAngleZeroed().lt(MoPrefs.wristNominalRevLimit.get());
    }

    public void disableWristNominalReverseLimit() {
        nominalReverseLimitEnabled.setBoolean(false);
        elevatorWristConfig.accept(config -> config.softLimit.reverseSoftLimit(0));
    }

    public void enableWristNominalReverseLimit() {
        nominalReverseLimitEnabled.setBoolean(true);
        elevatorWristConfig.accept(config -> config.softLimit.reverseSoftLimit(
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

    public Angle getWristAbsAngleZeroed() {
        wristAbsEncoderValue.mut_replace(wristAbsEncoder.getPosition());
        wristAbsEncoderValue.mut_minus(MoPrefs.wristAbsZero.get());
        return wristAbsEncoderValue;
    }

    private Angle getWristAngleFromHorizontal() {

        return wristAngleFromHorizontal
                .mut_replace(MoPrefs.wristHorizontal.get())
                .mut_minus(wristRelEncoder.getPosition());
        // return wristAngleFromHorizontal
        //         .mut_replace(wristRelEncoder.getPosition())
        //         .mut_minus(MoPrefs.wristHorizontal.get());
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

        var stowPos = ElevatorSetpointManager.getInstance().getSetpoint(ElevatorSetpoint.STOW);
        switch (controlMode.getSelected()) {
            case SMARTMOTION:
                wristTrapPid.setPositionReference(stowPos.wristAngle());
                break;
            case RAW_POSITION_PID:
                wristPositionPid.setPositionReference(stowPos.wristAngle());
                break;
            case DIRECT_VELOCITY:
                wristVelocityPid.setVelocityReference(Units.RotationsPerSecond.zero());
                break;
            case FALLBACK_DIRECT_POWER:
                elevatorWrist.setVoltage(0);
                break;
        }
    }

    public void disableReverseLimit() {
        elevatorAConfig.accept(config -> config.softLimit.reverseSoftLimitEnabled(false));
    }

    public void zeroElevator() {
        hasZero.setBoolean(true);
        elevatorRelEncoder.setPosition(Units.Centimeters.zero());
        elevatorAConfig.accept(
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

        Consumer<Distance> currModeElevatorPid;
        Consumer<Angle> currModeWristPid;

        if (controlMode.getSelected() == ElevatorControlMode.SMARTMOTION) {
            currModeElevatorPid = elevatorTrapPid::setPositionReference;
            currModeWristPid = wristPositionPid::setPositionReference; // wristTrapPid::setPositionReference;
        } else {
            currModeElevatorPid = elevatorPositionPid::setPositionReference;
            currModeWristPid = wristPositionPid::setPositionReference;
        }

        if (cutElevatorPower) {
            elevatorA.setVoltage(0);
        } else {
            currModeElevatorPid.accept(position.elevatorDistance);
        }
        currModeWristPid.accept(position.wristAngle);
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
        if (elevatorAConfig.checkForBrownout()) {
            hasZero.setBoolean(false);
            elevatorAConfig.accept(
                    config -> config.softLimit.reverseSoftLimitEnabled(false).forwardSoftLimitEnabled(false));
        }
        elevatorBConfig.checkForBrownout();
        if (elevatorWristConfig.checkForBrownout()) {
            reZeroWrist();
        }
    }
}
