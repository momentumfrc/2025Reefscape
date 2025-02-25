package frc.robot.molib.encoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.molib.MoSparkConfigurator;
import java.util.function.Consumer;

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
        return forSparkRelative(spark.getEncoder(), internalEncoderUnits, MoSparkConfigurator.forSparkBase(spark));
    }

    public static MoDistanceEncoder forSparkRelative(
            RelativeEncoder encoder,
            DistanceUnit internalEncoderUnits,
            Consumer<Consumer<SparkBaseConfig>> configurator) {
        return new MoDistanceEncoder(new RevRelativeEncoder(encoder, configurator), internalEncoderUnits);
    }

    public static MoDistanceEncoder forSparkAbsolute(SparkBase spark, DistanceUnit internalEncoderUnits) {
        return forSparkAbsolute(
                spark.getAbsoluteEncoder(), internalEncoderUnits, MoSparkConfigurator.forSparkBase(spark));
    }

    public static MoDistanceEncoder forSparkAbsolute(
            AbsoluteEncoder encoder,
            DistanceUnit internalEncoderUnits,
            Consumer<Consumer<SparkBaseConfig>> configurator) {
        return new MoDistanceEncoder(new RevAbsoluteEncoder(encoder, configurator), internalEncoderUnits);
    }

    public static MoDistanceEncoder forSparkAnalog(SparkBase spark, DistanceUnit internalEncoderUnits) {
        return forSparkAnalog(spark.getAnalog(), internalEncoderUnits, MoSparkConfigurator.forSparkBase(spark));
    }

    public static MoDistanceEncoder forSparkAnalog(
            SparkAnalogSensor sensor,
            DistanceUnit internalEncoderUnits,
            Consumer<Consumer<SparkBaseConfig>> configurator) {
        return new MoDistanceEncoder(new RevAnalogSensorEncoder(sensor, configurator), internalEncoderUnits);
    }

    public static MoDistanceEncoder forTalonFx(TalonFX talon, DistanceUnit internalEncoderUnits) {
        return new MoDistanceEncoder(new TalonFxEncoder(talon), internalEncoderUnits);
    }
}
