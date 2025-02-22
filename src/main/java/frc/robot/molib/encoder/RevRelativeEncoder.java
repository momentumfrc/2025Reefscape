package frc.robot.molib.encoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import java.util.function.Supplier;

public class RevRelativeEncoder implements MoEncoder.Encoder {
    public static final TimeUnit VELOCITY_BASE_UNIT = Units.Minute;

    private final SparkBase spark;
    private final Supplier<SparkBaseConfig> configSupplier;
    private final RelativeEncoder encoder;

    public RevRelativeEncoder(SparkBase spark, Supplier<SparkBaseConfig> configSupplier) {
        this.spark = spark;
        this.encoder = spark.getEncoder();
        this.configSupplier = configSupplier;

        this.setPositionFactor(1);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void setPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setPositionFactor(double factor) {
        SparkBaseConfig config = configSupplier.get();
        config.encoder.positionConversionFactor(factor).velocityConversionFactor(factor);
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public TimeUnit getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }

    @Override
    public void setInverted(boolean inverted) {
        SparkBaseConfig config = configSupplier.get();
        config.encoder.inverted(inverted);
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
