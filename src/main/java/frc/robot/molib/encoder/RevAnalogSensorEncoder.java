package frc.robot.molib.encoder;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import frc.robot.utils.MoUtils;
import java.util.function.Supplier;

public class RevAnalogSensorEncoder implements MoEncoder.Encoder {
    public static TimeUnit VELOCITY_BASE_UNIT = Units.Seconds;

    private final SparkBase spark;
    private final Supplier<SparkBaseConfig> configSupplier;
    private final SparkAnalogSensor sensor;

    public RevAnalogSensorEncoder(SparkBase spark, Supplier<SparkBaseConfig> configSupplier) {
        this.spark = spark;
        this.sensor = spark.getAnalog();
        this.configSupplier = configSupplier;

        this.setPositionFactor(1);
    }

    @Override
    public double getPosition() {
        return sensor.getPosition();
    }

    @Override
    public void setPosition(double position) {
        throw new UnsupportedOperationException("Cannot set position on an absolute encoder");
    }

    @Override
    public double getVelocity() {
        return sensor.getVelocity();
    }

    @Override
    public void setPositionFactor(double factor) {
        SparkBaseConfig config = MoUtils.getSparkConfig(spark);
        config.analogSensor.positionConversionFactor(factor).velocityConversionFactor(factor);
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public TimeUnit getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }

    @Override
    public void setInverted(boolean inverted) {
        SparkBaseConfig config = MoUtils.getSparkConfig(spark);

        config.analogSensor.inverted(inverted);

        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
