package frc.robot.molib.encoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utils.MoUtils;
import java.util.function.Supplier;

public class MoDistanceEncoder extends MoEncoder<DistanceUnit, LinearVelocityUnit> {
    MoDistanceEncoder(Encoder encoder, DistanceUnit internalEncoderUnits) {
        super(encoder, internalEncoderUnits);
    }

    @Override
    public Distance getPosition() {
        return (Distance) super.getPosition();
    }

    @Override
    public LinearVelocity getVelocity() {
        return (LinearVelocity) super.getVelocity();
    }

    public static MoDistanceEncoder forSparkRelative(SparkBase spark, DistanceUnit internalEncoderUnits) {
        return forSparkRelative(spark, internalEncoderUnits, () -> MoUtils.getSparkConfig(spark));
    }

    public static MoDistanceEncoder forSparkRelative(
            SparkBase spark, DistanceUnit internalEncoderUnits, Supplier<SparkBaseConfig> configSupplier) {
        return new MoDistanceEncoder(new RevRelativeEncoder(spark, configSupplier), internalEncoderUnits);
    }

    public static MoDistanceEncoder forSparkAbsolute(SparkBase spark, DistanceUnit internalEncoderUnits) {
        return forSparkAbsolute(spark, internalEncoderUnits, () -> MoUtils.getSparkConfig(spark));
    }

    public static MoDistanceEncoder forSparkAbsolute(
            SparkBase spark, DistanceUnit internalEncoderUnits, Supplier<SparkBaseConfig> configSupplier) {
        return new MoDistanceEncoder(new RevAbsoluteEncoder(spark, configSupplier), internalEncoderUnits);
    }

    public static MoDistanceEncoder forSparkAnalog(SparkBase spark, DistanceUnit internalEncoderUnits) {
        return forSparkAnalog(spark, internalEncoderUnits, () -> MoUtils.getSparkConfig(spark));
    }

    public static MoDistanceEncoder forSparkAnalog(
            SparkBase spark, DistanceUnit internalEncoderUnits, Supplier<SparkBaseConfig> configSupplier) {
        return new MoDistanceEncoder(new RevAnalogSensorEncoder(spark, configSupplier), internalEncoderUnits);
    }

    public static MoDistanceEncoder forTalonFx(TalonFX talon, DistanceUnit internalEncoderUnits) {
        return new MoDistanceEncoder(new TalonFxEncoder(talon), internalEncoderUnits);
    }
}
