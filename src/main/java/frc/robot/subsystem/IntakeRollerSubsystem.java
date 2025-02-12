package frc.robot.subsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;

public class IntakeRollerSubsystem extends SubsystemBase {
    private static final Current SMART_CURRENT_LIMIT = Units.Amps.of(20);

    private final SparkMax roller;
    private final RelativeEncoder encoder;

    private GenericEntry isBallHeld = MoShuffleboard.getInstance()
            .intakeTab
            .add("Ball Held?", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();

    private MutAngularVelocity encoderVelocity = Units.RotationsPerSecond.mutable(0);

    public IntakeRollerSubsystem() {
        super("Ground Intake Rollers");

        this.roller = new SparkMax(Constants.INTAKE_ROLLER.address(), MotorType.kBrushed);
        this.roller.configure(
                new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit((int)
                        SMART_CURRENT_LIMIT.in(Units.Amps)),
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        this.encoder = roller.getEncoder();

        MoShuffleboard.getInstance().intakeTab.addDouble("Roller Velocity (RPS)", () -> getIntakeVelocity()
                .in(Units.RotationsPerSecond));

        MoShuffleboard.getInstance().intakeTab.add(this);
    }

    public void rollerIntake() {
        roller.setVoltage(MoPrefs.intakeRollerPower.get().in(Units.Volts));
    }

    public void rollerShoot() {
        roller.setVoltage(-1 * MoPrefs.intakeRollerPower.get().in(Units.Volts));
    }

    public void stopRollerMotor() {
        roller.set(0);
    }

    public boolean isBallHeld() {
        return isBallHeld.getBoolean(false);
    }

    public void setBallHeld(boolean isHeld) {
        isBallHeld.setBoolean(isHeld);
    }

    public AngularVelocity getIntakeVelocity() {
        return encoderVelocity.mut_replace(encoder.getVelocity(), Units.RPM);
    }
}
