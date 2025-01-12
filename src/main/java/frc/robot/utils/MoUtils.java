package frc.robot.utils;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class MoUtils {

    public static SparkBaseConfig getSparkConfig(SparkBase spark) {
        if (spark instanceof SparkMax) {
            return new SparkMaxConfig();
        } else if (spark instanceof SparkFlex) {
            return new SparkFlexConfig();
        } else {
            throw new IllegalArgumentException("Unsupported SparkBase subclass");
        }
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
