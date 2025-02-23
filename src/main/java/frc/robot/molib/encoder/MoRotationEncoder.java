package frc.robot.molib.encoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.utils.MoUtils;
import java.util.function.Supplier;

public class MoRotationEncoder extends MoEncoder<AngleUnit, AngularVelocityUnit> {
    MoRotationEncoder(Encoder encoder, AngleUnit internalEncoderUnits) {
        super(encoder, internalEncoderUnits);
    }

    @Override
    public Angle getPosition() {
        return (Angle) super.getPosition();
    }

    @Override
    public AngularVelocity getVelocity() {
        return (AngularVelocity) super.getVelocity();
    }

    public static MoRotationEncoder forSparkRelative(SparkBase spark, AngleUnit internalEncoderUnits) {
        return forSparkRelative(spark, internalEncoderUnits, () -> MoUtils.getSparkConfig(spark));
    }

    public static MoRotationEncoder forSparkRelative(
            SparkBase spark, AngleUnit internalEncoderUnits, Supplier<SparkBaseConfig> configSupplier) {
        return new MoRotationEncoder(new RevRelativeEncoder(spark, configSupplier), internalEncoderUnits);
    }

    public static MoRotationEncoder forSparkAbsolute(SparkBase spark, AngleUnit internalEncoderUnits) {
        return forSparkAbsolute(spark, internalEncoderUnits, () -> MoUtils.getSparkConfig(spark));
    }

    public static MoRotationEncoder forSparkAbsolute(
            SparkBase spark, AngleUnit internalEncoderUnits, Supplier<SparkBaseConfig> configSupplier) {
        return new MoRotationEncoder(new RevAbsoluteEncoder(spark, configSupplier), internalEncoderUnits);
    }

    public static MoRotationEncoder forSparkAnalog(SparkBase spark, AngleUnit internalEncoderUnits) {
        return forSparkAnalog(spark, internalEncoderUnits, () -> MoUtils.getSparkConfig(spark));
    }

    public static MoRotationEncoder forSparkAnalog(
            SparkBase spark, AngleUnit internalEncoderUnits, Supplier<SparkBaseConfig> configSupplier) {
        return new MoRotationEncoder(new RevAnalogSensorEncoder(spark, configSupplier), internalEncoderUnits);
    }

    public static MoRotationEncoder forTalonFx(TalonFX talon, AngleUnit internalEncoderUnits) {
        return new MoRotationEncoder(new TalonFxEncoder(talon), internalEncoderUnits);
    }
}
