package frc.robot.molib.encoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import frc.robot.utils.MoUtils;

public class RevAbsoluteEncoder implements MoEncoder.Encoder {
    public static TimeUnit VELOCITY_BASE_UNIT = Units.Seconds;

    private SparkBase spark;
    private AbsoluteEncoder encoder;

    public RevAbsoluteEncoder(SparkBase spark) {
        this.spark = spark;
        this.encoder = spark.getAbsoluteEncoder();

        setPositionFactor(1);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void setPosition(double position) {
        throw new UnsupportedOperationException("Cannot set position on an absolute encoder");
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setPositionFactor(double factor) {
        SparkBaseConfig config = MoUtils.getSparkConfig(spark);
        config.absoluteEncoder.positionConversionFactor(factor).velocityConversionFactor(factor);
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public TimeUnit getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }

    @Override
    public void setInverted(boolean inverted) {
        SparkBaseConfig config = MoUtils.getSparkConfig(spark);
        config.inverted(inverted);
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
