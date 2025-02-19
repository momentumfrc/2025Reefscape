package frc.robot.molib.encoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

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
        return new MoDistanceEncoder(new RevRelativeEncoder(spark), internalEncoderUnits);
    }

    public static MoDistanceEncoder forSparkAbsolute(SparkBase spark, DistanceUnit internalEncoderUnits) {
        return new MoDistanceEncoder(new RevAbsoluteEncoder(spark), internalEncoderUnits);
    }

    public static MoDistanceEncoder forSparkAnalog(SparkBase spark, DistanceUnit internalEncoderUnits) {
        return new MoDistanceEncoder(new RevAnalogSensorEncoder(spark), internalEncoderUnits);
    }

    public static MoDistanceEncoder forTalonFx(TalonFX talon, DistanceUnit internalEncoderUnits) {
        return new MoDistanceEncoder(new TalonFxEncoder(talon), internalEncoderUnits);
    }
}
