package frc.robot.molib.encoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import java.util.function.Consumer;

public class RevAbsoluteEncoder implements MoEncoder.Encoder {
    public static TimeUnit VELOCITY_BASE_UNIT = Units.Seconds;

    private final Consumer<Consumer<SparkBaseConfig>> configurator;
    private final AbsoluteEncoder encoder;

    public RevAbsoluteEncoder(AbsoluteEncoder encoder, Consumer<Consumer<SparkBaseConfig>> configurator) {
        this.encoder = encoder;
        this.configurator = configurator;

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
        configurator.accept(config ->
                config.absoluteEncoder.positionConversionFactor(factor).velocityConversionFactor(factor));
    }

    @Override
    public TimeUnit getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }

    @Override
    public void setInverted(boolean inverted) {
        configurator.accept(config -> config.absoluteEncoder.inverted(inverted));
    }
}
