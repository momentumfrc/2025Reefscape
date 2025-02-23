package frc.robot.molib.encoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import frc.robot.molib.MoSparkConfigurator;
import frc.robot.molib.MoUnits;
import java.util.function.Consumer;

/**
 * Wraps an encoder, keeping track of the encoder's internal units.
 */
public class MoEncoder<Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> {
    public static interface Encoder {
        public double getPosition();

        public void setPosition(double position);

        public double getVelocity();

        public void setInverted(boolean inverted);

        /**
         * Set the factor used internally by the encoder to scale encoder ticks to the desired output units.
         * <p>
         * Note: the factor is in units of internalEncoderUnits / encoder_tick.
         * @param factor The factor to be internally multiplied by encoder ticks.
         */
        public void setPositionFactor(double factor);

        /**
         * Velocity is measured in distance per time. However, the dividing time unit is different across different
         * encoders. For example, CTRE returns velocity in units of internalEncoderUnits per *second*, but
         * Rev returns velocity in units of internalEncoderUnits per *minute*. So, for a CTRE encoder this method
         * would return Units.Second, but for a Rev encoder this method would return Units.Minute.
         * @return The base time unit of the velocity returned by this encoder.
         */
        public TimeUnit getVelocityBaseUnit();
    }

    private final Encoder encoder;

    /**
     * The units of the value returned by encoder.getPosition(), assuming the encoder has internally applied the set
     * position factor.
     */
    private final Dim internalEncoderUnits;

    /*
     * The units of the value returned by encoder.getVelocity(), assuming the encoder has internally applied the set
     * position factor.
     */
    private final VDim internalEncoderVelocityUnits;

    private final MutableMeasure<Dim, ?, ?> mut_pos;
    private final MutableMeasure<VDim, ?, ?> mut_vel;

    @SuppressWarnings("unchecked")
    MoEncoder(Encoder encoder, Dim internalEncoderUnits) {
        this.encoder = encoder;
        this.internalEncoderUnits = internalEncoderUnits;
        this.internalEncoderVelocityUnits = (VDim) internalEncoderUnits.per(encoder.getVelocityBaseUnit());

        mut_pos = (MutableMeasure<Dim, ?, ?>) internalEncoderUnits.mutable(0);
        mut_vel = (MutableMeasure<VDim, ?, ?>) internalEncoderVelocityUnits.mutable(0);
    }

    public Dim getInternalEncoderUnits() {
        return internalEncoderUnits;
    }

    public VDim getInternalEncoderVelocityUnits() {
        return internalEncoderVelocityUnits;
    }

    public Encoder getEncoder() {
        return encoder;
    }

    public void setConversionFactor(Measure<PerUnit<DimensionlessUnit, Dim>> mechanismConversionFactor) {
        /*
             * We need to set the position factor such that, when we call getPosition(), the value returned is in the
        desired
             * units as specified by internalEncoderUnits. We can model the internal encoder calculation as such:
             * [ p encoder_ticks ] * [pos_factor internalEncoderUnits / encoder_ticks] = [p internalEncoderUnits]
             * So, to get the output of the encoder in units of internalEncoderUnits, we need to calculate the pos_factor
             * in units of internalEncoderUnits per encoder_ticks, then set that as the positionConversionFactor.
             */
        double positionFactor = 1 / mechanismConversionFactor.in(MoUnits.EncoderTicks.per(internalEncoderUnits));

        encoder.setPositionFactor(positionFactor);
    }

    public Measure<Dim> getPosition() {
        return mut_pos.mut_replace(encoder.getPosition(), internalEncoderUnits);
    }

    public void setPosition(Measure<Dim> position) {
        encoder.setPosition(position.in(internalEncoderUnits));
    }

    public Measure<VDim> getVelocity() {
        return mut_vel.mut_replace(encoder.getVelocity(), internalEncoderVelocityUnits);
    }

    public double getPositionInEncoderUnits() {
        return getPosition().in(internalEncoderUnits);
    }

    public double getVelocityInEncoderUnits() {
        return getVelocity().in(internalEncoderVelocityUnits);
    }

    public void setInverted(boolean inverted) {
        encoder.setInverted(inverted);
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> MoEncoder<Dim, VDim> forSparkRelative(
            SparkBase spark, Dim internalEncoderUnits) {
        return forSparkRelative(spark.getEncoder(), internalEncoderUnits, MoSparkConfigurator.forSparkBase(spark));
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> MoEncoder<Dim, VDim> forSparkRelative(
            RelativeEncoder encoder, Dim internalEncoderUnits, Consumer<Consumer<SparkBaseConfig>> configurator) {
        return new MoEncoder<Dim, VDim>(new RevRelativeEncoder(encoder, configurator), internalEncoderUnits);
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> MoEncoder<Dim, VDim> forSparkAbsolute(
            SparkBase spark, Dim internalEncoderUnits) {
        return forSparkAbsolute(
                spark.getAbsoluteEncoder(), internalEncoderUnits, MoSparkConfigurator.forSparkBase(spark));
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> MoEncoder<Dim, VDim> forSparkAbsolute(
            AbsoluteEncoder encoder, Dim internalEncoderUnits, Consumer<Consumer<SparkBaseConfig>> configurator) {
        return new MoEncoder<Dim, VDim>(new RevAbsoluteEncoder(encoder, configurator), internalEncoderUnits);
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> MoEncoder<Dim, VDim> forSparkAnalog(
            SparkBase spark, Dim internalEncoderUnits) {
        return forSparkAnalog(spark.getAnalog(), internalEncoderUnits, MoSparkConfigurator.forSparkBase(spark));
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> MoEncoder<Dim, VDim> forSparkAnalog(
            SparkAnalogSensor sensor, Dim internalEncoderUnits, Consumer<Consumer<SparkBaseConfig>> configurator) {
        return new MoEncoder<Dim, VDim>(new RevAnalogSensorEncoder(sensor, configurator), internalEncoderUnits);
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> MoEncoder<Dim, VDim> forTalonFx(
            TalonFX talon, Dim internalEncoderUnits) {
        return new MoEncoder<Dim, VDim>(new TalonFxEncoder(talon), internalEncoderUnits);
    }
}
