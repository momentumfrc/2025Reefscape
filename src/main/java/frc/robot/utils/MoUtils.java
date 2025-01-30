package frc.robot.utils;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.molib.encoder.MoDistanceEncoder;
import frc.robot.molib.encoder.MoRotationEncoder;

public class MoUtils {
    private static final double ENCODER_ZERO_ZONE = 0.2;

    public static void setupRelativeEncoder(
            MoRotationEncoder encoder,
            Angle absPos,
            Measure<AngleUnit> absZero,
            Measure<PerUnit<DimensionlessUnit, AngleUnit>> ratio) {
        encoder.setConversionFactor(ratio);

        double pos = absPos.in(Units.Rotations);
        pos = (pos + 1 - absZero.in(Units.Rotations)) % 1;
        if (pos > (1 - ENCODER_ZERO_ZONE)) {
            pos -= 1;
        }
        encoder.setPosition(Units.Rotations.of(pos));
    }

    public static void setupRelativeEncoder(
            MoDistanceEncoder encoder,
            Angle absPos,
            Measure<AngleUnit> absZero,
            Measure<PerUnit<DimensionlessUnit, DistanceUnit>> ratio) {
        encoder.setConversionFactor(ratio);

        double pos = absPos.in(Units.Rotations);
        pos = (pos + 1 - absZero.in(Units.Rotations)) % 1;
        if (pos > (1 - ENCODER_ZERO_ZONE)) {
            pos -= 1;
        }
        encoder.setPosition(Units.Centimeters.of(pos));
    }

    public static SparkBaseConfig getSparkConfig(SparkBase spark) {
        if (spark instanceof SparkMax) {
            return new SparkMaxConfig();
        } else if (spark instanceof SparkFlex) {
            return new SparkFlexConfig();
        } else {
            throw new IllegalArgumentException("Unsupported SparkBase subclass");
        }
    }

    public static double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    public static double curve(double val, double curve) {
        if (curve == 0) {
            return val;
        }

        return Math.signum(val) * Math.pow(Math.abs(val), curve);
    }

    private MoUtils() {
        throw new UnsupportedOperationException("MoUtils is a static class");
    }
}
