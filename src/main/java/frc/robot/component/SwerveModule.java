package frc.robot.component;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.momentum4999.molib.MoSparkConfigurator;
import com.momentum4999.molib.MoUnits;
import com.momentum4999.molib.encoder.MoDistanceEncoder;
import com.momentum4999.molib.encoder.MoRotationEncoder;
import com.momentum4999.molib.pid.MoSparkMaxPID;
import com.momentum4999.molib.pid.MoTalonFxPID;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutLinearVelocity;
import frc.robot.molib.prefs.UnitPref;
import frc.robot.utils.TunerUtils;

public class SwerveModule {
    // The Thrifty absolute magnetic encoder outputs a voltage between 0-5v throughout 1 rotation, but it's scaled
    // down to 1-3.3v by the SparkMax data port breakout board. Thus, when the mechanism travels 1 rotation, the
    // absolute encoder will travel through 3.3 "Encoder Ticks" (which happen to correspond to volts).
    private static final Measure<PerUnit<DimensionlessUnit, AngleUnit>> ABSOLUTE_ENCODER_SCALE =
            MoUnits.EncoderTicksPerRotation.ofNative(3.3);

    private static final double MOTOR_UNPOWERED_SPEED = 0.05;

    private final String key;
    public final SparkMax turnMotor;
    public final TalonFX driveMotor;

    public final MoSparkConfigurator turnMotorConfig;

    public final MoRotationEncoder absoluteEncoder;
    public final MoRotationEncoder relativeEncoder;

    public final MoDistanceEncoder distEncoder;

    public final MoSparkMaxPID<AngleUnit, AngularVelocityUnit> turnPID;
    private final MoTalonFxPID<DistanceUnit, LinearVelocityUnit> drivePID;

    private final UnitPref<AngleUnit> encoderZero;
    private final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> encoderRotScale;

    private final MutAngle mut_angleSetpoint = Units.Rotations.mutable(0);
    private final MutLinearVelocity mut_velocitySetpoint = Units.MetersPerSecond.mutable(0);

    public SwerveModule(
            String key,
            SparkMax turnMotor,
            TalonFX driveMotor,
            UnitPref<AngleUnit> encoderZero,
            UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> encoderRotScale,
            UnitPref<PerUnit<DimensionlessUnit, DistanceUnit>> encoderDistScale) {
        this.key = key;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        this.encoderZero = encoderZero;
        this.encoderRotScale = encoderRotScale;

        this.turnMotorConfig = MoSparkConfigurator.forSparkMax(turnMotor);

        this.driveMotor.setNeutralMode(NeutralModeValue.Brake);

        turnMotorConfig.accept(config -> {
            config.idleMode(IdleMode.kBrake);
            config.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI).positionWrappingEnabled(true);
        });

        this.absoluteEncoder = MoRotationEncoder.forSparkAnalog(turnMotor, Units.Rotations);
        this.absoluteEncoder.setConversionFactor(ABSOLUTE_ENCODER_SCALE);

        relativeEncoder = MoRotationEncoder.forSparkRelative(turnMotor, Units.Radians);

        distEncoder = MoDistanceEncoder.forTalonFx(driveMotor, Units.Meters);
        encoderDistScale.subscribe(scale -> distEncoder.setConversionFactor(scale), true);

        this.turnPID =
                new MoSparkMaxPID<>(MoSparkMaxPID.Type.POSITION, turnMotor, ClosedLoopSlot.kSlot0, relativeEncoder);
        this.drivePID =
                new MoTalonFxPID<>(MoTalonFxPID.Type.VELOCITY, driveMotor, distEncoder.getInternalEncoderUnits());

        TunerUtils.forMoSparkMax(turnPID, key + "_turn");
        TunerUtils.forMoTalonFx(drivePID, key + "_drive");

        encoderZero.subscribe(
                zero -> this.setupRelativeEncoder(absoluteEncoder.getPosition(), zero, encoderRotScale.get()));
        encoderRotScale.subscribe(
                scale -> this.setupRelativeEncoder(absoluteEncoder.getPosition(), encoderZero.get(), scale));
        setupRelativeEncoder();
    }

    private boolean areMotorsPowered() {
        return Math.abs(driveMotor.get()) > MOTOR_UNPOWERED_SPEED && Math.abs(turnMotor.get()) > MOTOR_UNPOWERED_SPEED;
    }

    public void setupRelativeEncoder() {
        setupRelativeEncoder(absoluteEncoder.getPosition(), encoderZero.get(), encoderRotScale.get());
    }

    public void setRelativePosition() {
        if (!areMotorsPowered()) {
            setRelativePosition(absoluteEncoder.getPosition(), encoderZero.get());
        }
    }

    private void setupRelativeEncoder(
            Measure<AngleUnit> absPos,
            Measure<AngleUnit> absZero,
            Measure<PerUnit<DimensionlessUnit, AngleUnit>> scale) {
        relativeEncoder.setConversionFactor(scale);
        setRelativePosition(absPos, absZero);
    }

    private void setRelativePosition(Measure<AngleUnit> absPos, Measure<AngleUnit> absZero) {
        double rots = absPos.in(Units.Rotations);
        rots = (rots + 1 - absZero.in(Units.Rotations)) % 1;
        relativeEncoder.setPosition(Units.Rotations.of(rots));
    }

    public void drive(SwerveModuleState state) {
        state.optimize(new Rotation2d(relativeEncoder.getPosition()));
        turnPID.setPositionReference(
                mut_angleSetpoint.mut_replace(MathUtil.angleModulus(state.angle.getRadians()), Units.Radians));

        drivePID.setVelocityReference(
                mut_velocitySetpoint.mut_replace(state.speedMetersPerSecond, Units.MetersPerSecond));
    }

    public void directDrive(double turnSpeed, double driveSpeed) {
        turnMotor.set(turnSpeed);
        driveMotor.setControl(new DutyCycleOut(driveSpeed));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(distEncoder.getPosition(), new Rotation2d(relativeEncoder.getPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(distEncoder.getVelocity(), new Rotation2d(relativeEncoder.getPosition()));
    }

    public String getKey() {
        return key;
    }

    @Override
    public String toString() {
        return String.format("SwerveModule(key=\"%s\")", this.key);
    }
}
