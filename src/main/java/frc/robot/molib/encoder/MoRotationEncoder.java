package frc.robot.molib.encoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

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
        return new MoRotationEncoder(new RevRelativeEncoder(spark), internalEncoderUnits);
    }

    public static MoRotationEncoder forSparkAbsolute(SparkBase spark, AngleUnit internalEncoderUnits) {
        return new MoRotationEncoder(new RevAbsoluteEncoder(spark), internalEncoderUnits);
    }

    public static MoRotationEncoder forSparkAnalog(SparkBase spark, AngleUnit internalEncoderUnits) {
        return new MoRotationEncoder(new RevAnalogSensorEncoder(spark), internalEncoderUnits);
    }

    public static MoRotationEncoder forTalonFx(TalonFX talon, AngleUnit internalEncoderUnits) {
        return new MoRotationEncoder(new TalonFxEncoder(talon), internalEncoderUnits);
    }
}
