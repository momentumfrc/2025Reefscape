package frc.robot.molib.prefs;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.molib.MoUnits;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.HashSet;
import java.util.Set;

/** Robot preferences, accessible through Shuffleboard */
public class MoPrefs {

    // ---------- Drive ----------
    public static final Pref<Double> inputDeadzone = unitlessDoublePref("Input Deadzone", 0.05);
    public static final Pref<Double> inputCurve = unitlessDoublePref("Input Curve", 1.5);
    public static final Pref<Double> inputTurnCurve = unitlessDoublePref("Input Turn Curve", 3);
    public static final TimeUnitPref driveRampTime = secondsPref("Drive Normal Ramp Time", Units.Seconds.of(0.1));
    public static final TimeUnitPref driveRampTimeElevatorExtended =
            secondsPref("Drive Elevator Extended Ramp Time", Units.Seconds.of(1.5));

    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> swerveRotScale =
            encoderTicksPerRotationPref("SWRV ROT Scale", MoUnits.EncoderTicksPerRotation.ofNative(12.8));
    public static final UnitPref<PerUnit<DimensionlessUnit, DistanceUnit>> swerveDistScale =
            encoderTicksPerMeterPref("SWRV DIST Scale", MoUnits.EncoderTicksPerMeter.ofNative(21.49));
    public static final AngleUnitPref swerveFLZero = rotationsPref("SWRV Zero FL", Units.Rotations.of(0.186));
    public static final AngleUnitPref swerveFRZero = rotationsPref("SWRV Zero FR", Units.Rotations.of(0.48));
    public static final AngleUnitPref swerveRLZero = rotationsPref("SWRV Zero RL", Units.Rotations.of(0.895));
    public static final AngleUnitPref swerveRRZero = rotationsPref("SWRV Zero RR", Units.Rotations.of(0.968));

    /**
     * The yaw offset between "forward" on the robot and "angle zero" on the gyro
     */
    public static final AngleUnitPref navxYawOffset = rotationsPref("NavX Yaw Offset", Units.Rotations.zero());

    public static final LinearVelocityUnitPref swerveMaxLinearSpeed =
            metersPerSecPref("SWRV Max Linear Speed", Units.MetersPerSecond.of(5));
    public static final AngularVelocityUnitPref swerveMaxAngularSpeed =
            rotationsPerSecPref("SWRV Max Angular Speed", Units.RotationsPerSecond.of(1));

    // ---------- Elevator ----------
    public static final Pref<Double> elevatorRampTime = unitlessDoublePref("Elevator Ramp Time", 0.15);

    public static final UnitPref<PerUnit<DimensionlessUnit, DistanceUnit>> elevatorEncoderScale =
            encoderTicksPerCentimeterPref(
                    "Elevator Encoder Scale", MoUnits.EncoderTicksPerCentimeter.ofNative(1.785446));

    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> wristEncoderScale =
            encoderTicksPerRotationPref("Wrist Encoder Scale", MoUnits.EncoderTicksPerRotation.ofNative(150));

    public static final DimensionlessUnitPref elevatorSetpointVarianceThreshold =
            percentPref("Elevator Setpoint Variance Threshold", Units.Percent.of(3));

    public static final AngleUnitPref wristAbsZero = rotationsPref("Wrist Absolute Zero", Units.Rotations.of(0));

    public static final UnitPref<VoltageUnit> elevatorZeroPower =
            voltsPref("Elevator Zero Power", Units.Volts.of(0.25));
    public static final UnitPref<CurrentUnit> elevatorZeroCurrentThresh =
            ampsPref("Elevator Zero Current Thresh", Units.Amps.of(10));
    public static final TimeUnitPref elevatorZeroCurrentTime = secondsPref("Elevator Zero Time", Units.Seconds.of(0.5));

    public static final DistanceUnitPref elevatorMaxExtension =
            centimetersPref("Elevator Max Extension", Units.Centimeters.of(73.66));
    public static final AngleUnitPref wristMaxExtension =
            rotationsPref("Wrist Max Extension", Units.Rotations.of(0.25));

    public static final AngleUnitPref wristNominalRevLimit =
            rotationsPref("Wrist Nominal Reverse Limit", Units.Rotations.of(0.2));
    public static final AngleUnitPref wristHorizontal = rotationsPref("Wrist Horizontal", Units.Rotations.of(0.45));

    public static final UnitPref<LinearVelocityUnit> elevatorMaxRps =
            metersPerSecPref("Elevator Max Spd", Units.MetersPerSecond.of(0.5));
    public static final UnitPref<AngularVelocityUnit> wristMaxRps =
            rotationsPerSecPref("Wrist Max Speed", Units.RotationsPerSecond.of(0.5));

    public static final DimensionlessUnitPref endEffectorPower =
            percentPref("End Effector Power", Units.Percent.of(20));

    // ---------- Climber ----------
    public static final TimeUnitPref climberRachetLockoutTime =
            secondsPref("Climber Rachet Lockout", Units.Seconds.of(0.5));
    public static final DimensionlessUnitPref rachetEngagedServoPosition =
            percentPref("Climber Rachet Engaged Servo Pos", Units.Percent.zero());
    public static final DimensionlessUnitPref rachetDisengagedServoPosition =
            percentPref("Climber Rachet Disengaged Servo Pos", Units.Percent.of(100));

    public static final Pref<Double> climberFwdSoftLimit = unitlessDoublePref("Climber FWD Soft Limit", 10);
    public static final Pref<Double> climberRvsSoftLimit = unitlessDoublePref("Climber RVS Soft Limit", 0.1);
    public static final DimensionlessUnitPref climberZeroPwr = percentPref("Climber Zero Power", Units.Percent.of(10));

    public static final DimensionlessUnitPref climberMaxPwr = percentPref("Climber Max Power", Units.Percent.of(75));

    // ---------- Intake ----------
    public static final UnitPref<VoltageUnit> intakeWristPower = voltsPref("Intake Wrist Power", Units.Volts.of(8));
    public static final UnitPref<VoltageUnit> intakeRollerPower = voltsPref("Intake Roller Power", Units.Volts.of(10));
    public static final UnitPref<VoltageUnit> intakeRollerHoldPower =
            voltsPref("Intake Roller Hold Power", Units.Volts.of(0.25));

    public static final UnitPref<VoltageUnit> intakeWristHoldPower =
            voltsPref("Intake Hold Wrist Power", Units.Volts.of(1));
    public static final UnitPref<VoltageUnit> intakeWristSoftHoldForIntakePower =
            voltsPref("Intake Wrist Soft Hold For Intake Power", Units.Volts.of(0.01));

    public static final TimeUnitPref intakeWristTime = secondsPref("Intake Wrist Time", Units.Seconds.one());

    public static final UnitPref<CurrentUnit> intakeRollersSmartCurrentLimit =
            ampsPref("Intake Rollers Smart Current Limit", Units.Amps.of(20));
    public static final UnitPref<CurrentUnit> intakeWristSmartCurrentLimit =
            ampsPref("Intake Wrist Smart Current Limit", Units.Amps.of(20));

    public static final TimeUnitPref intakeRollerSpinupTime =
            secondsPref("Intake Roller Spinup Time", Units.Seconds.of(0.25));
    public static final UnitPref<AngularVelocityUnit> intakeVelocityThreshold =
            rotationsPerSecPref("Intake Roller Velocity Threshold", Units.RotationsPerSecond.of(5));
    public static final TimeUnitPref intakeRollerThesholdTime =
            secondsPref("Intake Roller Threshold Time", Units.Seconds.of(0.5));
    public static final TimeUnitPref intakeRollerExtakeTime =
            secondsPref("Intake Roller Extake Time", Units.Seconds.of(0.75));

    public static final UnitPref<VoltageUnit> intakeRollerProbePower = voltsPref("Intake Roller Probe Power", Units.Volts.of(3));

    // ---------- Auto ----------
    public static final LinearVelocityUnitPref autoMaxLinVel =
            metersPerSecPref("Auto Max Linear Velocity", Units.MetersPerSecond.of(1.5));
    public static final LinearAccelerationUnitPref autoMaxLinAccel =
            metersPerSecPerSecPref("Auto Max Linear Acceleration", Units.MetersPerSecondPerSecond.of(2));
    public static final AngularVelocityUnitPref autoMaxAngVel =
            rotationsPerSecPref("Auto Max Angular Velocity", Units.RotationsPerSecond.of(2));
    public static final AngularAccelerationUnitPref autoMaxAngAccel =
            rotationsPerSecPerSecPref("Auto Max Angular Acceleration", Units.RotationsPerSecondPerSecond.of(2));
    public static final DistanceUnitPref autoLeaveDist = metersPref("Auto Leave Distance", Units.Meters.of(1.5));
    public static final DimensionlessUnitPref autoFallbackSpd =
            percentPref("Auto Fallback Power", Units.Percent.of(10));
    public static final TimeUnitPref autoFallbackTime = secondsPref("Auto Fallback Time", Units.Seconds.of(4));
    public static final TimeUnitPref autoExtakePreloadTime =
            secondsPref("Auto Extake Preload Time", Units.Seconds.of(1));

    public static final DimensionlessUnitPref ledBrightness = percentPref("LED Brightness", Units.Percent.of(20));

    NetworkTable backingTable;

    private static MoPrefs instance;
    private StringPublisher typePublisher;

    static synchronized MoPrefs getInstance() {
        if (instance == null) {
            instance = new MoPrefs();
        }
        return instance;
    }

    public static void cleanUpPrefs() {
        MoPrefs instance = getInstance();

        HashSet<String> pref_keys = new HashSet<>();

        // Shouldn't remove the special field .type
        pref_keys.add(".type");

        for (Field f : MoPrefs.class.getFields()) {
            if (Modifier.isStatic(f.getModifiers())) {
                Object fo;

                try {
                    fo = f.get(instance);
                } catch (IllegalArgumentException | IllegalAccessException e) {
                    continue;
                }

                if (fo instanceof Pref fop) {
                    pref_keys.add(fop.getKey());
                } else if (fo instanceof UnitPref fop) {
                    pref_keys.add(fop.getKey());
                }
            }
        }

        Set<String> table_keys = instance.backingTable.getKeys();

        System.out.println("****** Clean up MoPrefs ******");
        for (String key : table_keys) {
            if (!pref_keys.contains(key)) {
                System.out.format("Remove unused pref \"%s\"\n", key);

                Topic topic = instance.backingTable.getTopic(key);
                topic.setPersistent(false);
                topic.setRetained(false);
            }
        }
    }

    private MoPrefs() {
        backingTable = NetworkTableInstance.getDefault().getTable("Preferences");

        String kSmartDashboardType = "RobotPreferences";
        typePublisher = backingTable
                .getStringTopic(".type")
                .publishEx(StringTopic.kTypeString, "{\"SmartDashboard\":\"" + kSmartDashboardType + "\"}");
        typePublisher.set(kSmartDashboardType);
    }

    private static Pref<Boolean> booleanPref(String key, boolean defaultValue) {
        return new Pref<>(key, defaultValue, NetworkTableValue::getBoolean, NetworkTableEntry::setBoolean);
    }

    private static Pref<Double> unitlessDoublePref(String key, double defaultValue) {
        return new Pref<>(key, defaultValue, NetworkTableValue::getDouble, NetworkTableEntry::setDouble);
    }

    private static AngleUnitPref rotationsPref(String key, Angle defaultValue) {
        return new AngleUnitPref(key, Units.Rotations, defaultValue);
    }

    private static AngleUnitPref degreesPref(String key, Angle defaultValue) {
        return new AngleUnitPref(key, Units.Degrees, defaultValue);
    }

    private static DistanceUnitPref metersPref(String key, Distance defaultValue) {
        return new DistanceUnitPref(key, Units.Meters, defaultValue);
    }

    private static DistanceUnitPref centimetersPref(String key, Distance defaultValue) {
        return new DistanceUnitPref(key, Units.Centimeters, defaultValue);
    }

    private static LinearVelocityUnitPref metersPerSecPref(String key, LinearVelocity defaultValue) {
        return new LinearVelocityUnitPref(key, Units.MetersPerSecond, defaultValue);
    }

    private static AngularVelocityUnitPref rotationsPerSecPref(String key, AngularVelocity defaultValue) {
        return new AngularVelocityUnitPref(key, Units.RotationsPerSecond, defaultValue);
    }

    private static TimeUnitPref secondsPref(String key, Time defaultValue) {
        return new TimeUnitPref(key, Units.Seconds, defaultValue);
    }

    private static DimensionlessUnitPref percentPref(String key, Dimensionless defaultValue) {
        return new DimensionlessUnitPref(key, Units.Percent, defaultValue);
    }

    private static LinearAccelerationUnitPref metersPerSecPerSecPref(String key, LinearAcceleration defaultValue) {
        return new LinearAccelerationUnitPref(key, Units.MetersPerSecondPerSecond, defaultValue);
    }

    private static AngularAccelerationUnitPref rotationsPerSecPerSecPref(String key, AngularAcceleration defaultValue) {
        return new AngularAccelerationUnitPref(key, Units.RotationsPerSecondPerSecond, defaultValue);
    }

    private static UnitPref<PerUnit<DimensionlessUnit, DistanceUnit>> encoderTicksPerCentimeterPref(
            String key, Measure<PerUnit<DimensionlessUnit, DistanceUnit>> defaultValue) {
        return new UnitPref<>(key, MoUnits.EncoderTicksPerCentimeter, defaultValue);
    }

    private static UnitPref<PerUnit<DimensionlessUnit, DistanceUnit>> encoderTicksPerMeterPref(
            String key, Measure<PerUnit<DimensionlessUnit, DistanceUnit>> defaultValue) {
        return new UnitPref<>(key, MoUnits.EncoderTicksPerMeter, defaultValue);
    }

    private static UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> encoderTicksPerRotationPref(
            String key, Measure<PerUnit<DimensionlessUnit, AngleUnit>> defaultValue) {
        return new UnitPref<>(key, MoUnits.EncoderTicksPerRotation, defaultValue);
    }

    private static UnitPref<CurrentUnit> ampsPref(String key, Measure<CurrentUnit> defaultValue) {
        return new UnitPref<>(key, Units.Amps, defaultValue);
    }

    private static UnitPref<VoltageUnit> voltsPref(String key, Measure<VoltageUnit> defaultValue) {
        return new UnitPref<>(key, Units.Volts, defaultValue);
    }
}
