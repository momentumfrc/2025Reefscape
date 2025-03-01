package frc.robot.molib;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class MoSparkConfigurator implements Consumer<Consumer<SparkBaseConfig>> {
    private final SparkBase spark;
    private final SparkBaseConfig config;

    private static final Map<SparkBase, MoSparkConfigurator> instances = new HashMap<>();

    private StringLogEntry brownoutLog;

    private MoSparkConfigurator(SparkBase spark, SparkBaseConfig config) {
        this.spark = spark;
        this.config = config;

        brownoutLog =
                new StringLogEntry(DataLogManager.getLog(), String.format("/Brownout/Spark_%d", spark.getDeviceId()));

        resetSafeParameters();
    }

    public static void persistAllParameters() {
        DriverStation.reportWarning("Persisting spark configurations", false);
        for (MoSparkConfigurator c : instances.values()) {
            c.persistConfiguration();
        }
    }

    public static MoSparkConfigurator forSparkMax(SparkMax spark) {
        return instances.computeIfAbsent(spark, s -> new MoSparkConfigurator(s, new SparkMaxConfig()));
    }

    public static MoSparkConfigurator forSparkFlex(SparkFlex spark) {
        return instances.computeIfAbsent(spark, s -> new MoSparkConfigurator(s, new SparkFlexConfig()));
    }

    public static MoSparkConfigurator forSparkBase(SparkBase spark) {
        if (spark instanceof SparkMax s) {
            return forSparkMax(s);
        } else if (spark instanceof SparkFlex s) {
            return forSparkFlex(s);
        } else {
            throw new IllegalArgumentException(
                    "Unknown SparkBase subclass " + spark.getClass().getCanonicalName());
        }
    }

    @Override
    public synchronized void accept(Consumer<SparkBaseConfig> configurator) {
        configurator.accept(config);
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void resetSafeParameters() {
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void persistConfiguration() {
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean checkForBrownout() {
        // var warnings = spark.getStickyWarnings();
        // if (!warnings.brownout && !warnings.hasReset) {
        //     return false;
        // }

        // spark.clearFaults();
        // DriverStation.reportWarning(String.format("Brownout on spark %d", spark.getDeviceId()), false);

        // brownoutLog.append("BROWNOUT");

        // spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // return true;
        return false;
    }
}
