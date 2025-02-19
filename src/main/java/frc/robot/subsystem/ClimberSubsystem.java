package frc.robot.subsystem;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
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

    // Rachet state is unknown at startup
    private RachetState currRachetState = null;

    public ClimberSubsystem() {
        this.leftSpark = new SparkMax(Constants.CLIMBER_LEFT.address(), MotorType.kBrushless);
        this.rightSpark = new SparkMax(Constants.CLIMBER_RIGHT.address(), MotorType.kBrushless);
        this.rachet = new Servo(Constants.CLIMBER_RACHET.port());

        this.leftSpark.configure(
                new SparkMaxConfig().idleMode(IdleMode.kBrake),
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        this.rightSpark.configure(
                new SparkMaxConfig().idleMode(IdleMode.kBrake).follow(leftSpark, true),
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        MoShuffleboard.getInstance().climberTab.add(this);
        MoShuffleboard.getInstance()
                .climberTab
                .addString("Rachet State", () -> currRachetState == null ? "UNKNOWN" : currRachetState.name())
                .withWidget(BuiltInWidgets.kTextView);
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

    @Override
    public void periodic() {
        if (currRachetState != null) {
            rachet.set(currRachetState.posPref.get().in(Units.Value));
        }
    }
}
