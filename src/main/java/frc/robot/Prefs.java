package frc.robot;

import com.momentum4999.molib.MoUnits;
import com.momentum4999.molib.prefs.AngleUnitPref;
import com.momentum4999.molib.prefs.AngularAccelerationUnitPref;
import com.momentum4999.molib.prefs.AngularVelocityUnitPref;
import com.momentum4999.molib.prefs.DimensionlessUnitPref;
import com.momentum4999.molib.prefs.DistanceUnitPref;
import com.momentum4999.molib.prefs.LinearAccelerationUnitPref;
import com.momentum4999.molib.prefs.LinearVelocityUnitPref;
import com.momentum4999.molib.prefs.MoPrefs;
import com.momentum4999.molib.prefs.Pref;
import com.momentum4999.molib.prefs.TimeUnitPref;
import com.momentum4999.molib.prefs.UnitPref;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;

public class Prefs extends MoPrefs {
    // ---------- Drive ----------
    public static final Pref<Double> inputDeadzone = unitlessDoublePref("Input Deadzone", 0.05);
    public static final Pref<Double> inputCurve = unitlessDoublePref("Input Curve", 1.5);
    public static final Pref<Double> inputTurnCurve = unitlessDoublePref("Input Turn Curve", 3);
    public static final TimeUnitPref driveRampTime = secondsPref("Drive Normal Ramp Time", Units.Seconds.of(0.1));
    public static final TimeUnitPref driveRampTimeElevatorExtended =
            secondsPref("Drive Elevator Extended Ramp Time", Units.Seconds.of(1.5));

    public static final Pref<Double> defaultThrottle = unitlessDoublePref("Default Xbox Throttle", 0.5);

    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> swerveRotScale =
            encoderTicksPerRotationPref("SWRV ROT Scale", MoUnits.EncoderTicksPerRotation.ofNative(12.8));
    public static final UnitPref<PerUnit<DimensionlessUnit, DistanceUnit>> swerveDistScale =
            encoderTicksPerMeterPref("SWRV DIST Scale", MoUnits.EncoderTicksPerMeter.ofNative(21.49));
    public static final AngleUnitPref swerveFLZero = rotationsPref("SWRV Zero FL", Units.Rotations.of(0.186));
    public static final AngleUnitPref swerveFRZero = rotationsPref("SWRV Zero FR", Units.Rotations.of(0.48));
    public static final AngleUnitPref swerveRLZero = rotationsPref("SWRV Zero RL", Units.Rotations.of(0.895));
    public static final AngleUnitPref swerveRRZero = rotationsPref("SWRV Zero RR", Units.Rotations.of(0.968));

    public static final DimensionlessUnitPref driveSlowSpeed = percentPref("Drive Slow Speed", Units.Percent.of(30));

    /**
     * The yaw offset between "forward" on the robot and "angle zero" on the gyro
     */
    public static final AngleUnitPref navxYawOffset = rotationsPref("NavX Yaw Offset", Units.Rotations.zero());

    public static final LinearVelocityUnitPref swerveMaxLinearSpeed =
            metersPerSecPref("SWRV Max Linear Speed", Units.MetersPerSecond.of(5));
    public static final AngularVelocityUnitPref swerveMaxAngularSpeed =
            rotationsPerSecPref("SWRV Max Angular Speed", Units.RotationsPerSecond.of(1));

    public static final DimensionlessUnitPref driveMaxThrottleClimberExtended =
            percentPref("SWRV Max Throttle Climber Extended", Units.Percent.of(30));

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

    public static final TimeUnitPref wristRampTime = secondsPref("Wrist Ramp Time", Units.Seconds.of(0.2));

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
    public static final Pref<Double> climberRetractedZone = unitlessDoublePref("Climber Retracted Zone", 5);

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

    public static final UnitPref<VoltageUnit> intakeRollerProbePower =
            voltsPref("Intake Roller Probe Power", Units.Volts.of(3));

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
    public static final DimensionlessUnitPref autoExtakeCoralPower =
            percentPref("Auto Extake Coral Power", Units.Percent.of(80));
    public static final DistanceUnitPref robotWidthWithBumpers =
            inchesPref("Robot Width with Bumpers", Units.Inches.of(37));
    public static final DistanceUnitPref autoReefApproachDistance =
            metersPref("Auto Reef Approach Distance", Units.Meters.of(0.5));

    public static final DimensionlessUnitPref ledBrightness = percentPref("LED Brightness", Units.Percent.of(20));
}
