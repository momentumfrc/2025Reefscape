package frc.robot.subsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.molib.prefs.UnitPref;
import java.util.EnumSet;

public class ClimberSubsystem extends SubsystemBase {
    public enum RachetState {
        ENGAGED(MoPrefs.rachetEngagedServoPosition),
        DISENGAGED(MoPrefs.rachetDisengagedServoPosition);

        public final UnitPref<DimensionlessUnit> posPref;

        private RachetState(UnitPref<DimensionlessUnit> posPref) {
            this.posPref = posPref;
        }
    }

    private final SparkMax leftSpark;
    private final SparkMax rightSpark;
    private final Servo rachet;

    private final SparkLimitSwitch reverseLimitSwitch;
    private final RelativeEncoder encoder;

    private final GenericEntry encodersZeroed = MoShuffleboard.getInstance()
            .climberTab
            .add("Encoders Zeroed", false)
            .getEntry();

    // Rachet state is unknown at startup
    private RachetState currRachetState = null;

    public ClimberSubsystem() {
        this.leftSpark = new SparkMax(Constants.CLIMBER_LEFT.address(), MotorType.kBrushless);
        this.rightSpark = new SparkMax(Constants.CLIMBER_RIGHT.address(), MotorType.kBrushless);
        this.rachet = new Servo(Constants.CLIMBER_RACHET.port());

        this.reverseLimitSwitch = leftSpark.getReverseLimitSwitch();
        this.encoder = leftSpark.getEncoder();

        var leftSparkConfig = new SparkMaxConfig();
        leftSparkConfig.idleMode(IdleMode.kBrake).inverted(false);
        leftSparkConfig
                .limitSwitch
                .forwardLimitSwitchEnabled(false)
                .reverseLimitSwitchEnabled(true)
                .reverseLimitSwitchType(Type.kNormallyOpen);
        leftSparkConfig
                .softLimit
                .forwardSoftLimit(MoPrefs.climberFwdSoftLimit.get())
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimit(MoPrefs.climberRvsSoftLimit.get())
                .reverseSoftLimitEnabled(false);

        MoPrefs.climberFwdSoftLimit.subscribe(fwdLimit -> {
            SparkMaxConfig config = new SparkMaxConfig();
            config.softLimit.forwardSoftLimit(fwdLimit);
            leftSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

        MoPrefs.climberRvsSoftLimit.subscribe(rvsLimit -> {
            SparkMaxConfig config = new SparkMaxConfig();
            config.softLimit.reverseSoftLimit(rvsLimit);
            leftSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

        this.leftSpark.configure(leftSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        SparkMaxConfig rightSparkConfig = new SparkMaxConfig();
        rightSparkConfig.idleMode(IdleMode.kBrake).follow(leftSpark, true);
        this.rightSpark.configure(rightSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        MoShuffleboard.getInstance().climberTab.add(this);
        MoShuffleboard.getInstance()
                .climberTab
                .addString("Rachet State", () -> currRachetState == null ? "UNKNOWN" : currRachetState.name())
                .withWidget(BuiltInWidgets.kTextView);
        MoShuffleboard.getInstance().climberTab.addDouble("Encoder", encoder::getPosition);
        MoShuffleboard.getInstance().climberTab.addBoolean("Limit Switch", reverseLimitSwitch::isPressed);

        // TODO: this syntax sucks. Make a nice MoTables wrapper
        var coastMotorsEntry = NetworkTableInstance.getDefault()
                .getTable("Shuffleboard/Climber")
                .getEntry("Coast Motors");
        coastMotorsEntry.setBoolean(false);
        coastMotorsEntry
                .getInstance()
                .addListener(coastMotorsEntry, EnumSet.of(NetworkTableEvent.Kind.kValueAll), coast -> {
                    System.out.println(coast.valueData.value.getBoolean());
                    var idleMode = coast.valueData.value.getBoolean() ? IdleMode.kCoast : IdleMode.kBrake;
                    SparkMaxConfig leftConfig = new SparkMaxConfig();
                    leftConfig.idleMode(idleMode);
                    leftSpark.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                    SparkMaxConfig rightConfig = new SparkMaxConfig();
                    rightConfig.idleMode(idleMode);
                    rightSpark.configure(
                            rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                });
    }

    public void extendClimber(double speed) {
        if (currRachetState == null || currRachetState != RachetState.DISENGAGED) {
            DriverStation.reportError("Ignoring request to extend climber as the rachet is not disengaged", false);
            leftSpark.set(0);
        } else {
            leftSpark.set(MathUtil.clamp(speed, 0, 1));
        }
    }

    public void retractClimber(double speed) {
        if (currRachetState == null || currRachetState != RachetState.ENGAGED) {
            DriverStation.reportError("Ignoring request to retract climber as the rachet is not engaged", false);
            leftSpark.set(0);
        } else {
            leftSpark.set(MathUtil.clamp(speed, -1, 0));
        }
    }

    public void idleClimber() {
        leftSpark.set(0);
    }

    public void moveRachet(RachetState state) {
        // We're still moving the rachet, we don't know what its current state is
        currRachetState = null;
        rachet.set(state.posPref.get().in(Units.Value));
    }

    public void finishMoveRachet(RachetState state) {
        currRachetState = state;
    }

    public RachetState getRachetState() {
        return currRachetState;
    }

    public void rezeroEncoder() {
        encodersZeroed.setBoolean(true);
        encoder.setPosition(0);
        SparkMaxConfig config = new SparkMaxConfig();
        config.softLimit.forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
        leftSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public boolean isZeroed() {
        return encodersZeroed.getBoolean(false);
    }

    public boolean isLimitSwitchPressed() {
        return reverseLimitSwitch.isPressed();
    }

    @Override
    public void periodic() {
        if (currRachetState != null) {
            rachet.set(currRachetState.posPref.get().in(Units.Value));
        }
    }
}
