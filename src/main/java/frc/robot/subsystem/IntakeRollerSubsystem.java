package frc.robot.subsystem;

import com.momentum4999.molib.MoSparkConfigurator;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Prefs;
import frc.robot.molib.MoShuffleboard;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final SparkMax roller;
    private final MoSparkConfigurator rollerConfig;
    private final RelativeEncoder encoder;

    private GenericEntry isBallHeld = MoShuffleboard.getInstance()
            .intakeTab
            .add("Ball Held?", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();

    private MutAngularVelocity encoderVelocity = Units.RotationsPerSecond.mutable(0);
    private MutCurrent rollerCurrent = Units.Amps.mutable(0);

    public IntakeRollerSubsystem() {
        super("Ground Intake Rollers");

        this.roller = new SparkMax(Constants.INTAKE_ROLLER.address(), MotorType.kBrushed);
        this.rollerConfig = MoSparkConfigurator.forSparkMax(roller);
        rollerConfig.accept(config -> {
            config.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit((int)
                    Prefs.intakeRollersSmartCurrentLimit.get().in(Units.Amps));
            config.encoder.inverted(true);
        });

        Prefs.intakeRollersSmartCurrentLimit.subscribe(
                limit -> rollerConfig.accept(config -> config.smartCurrentLimit((int) limit.in(Units.Amps))));

        this.encoder = roller.getEncoder();

        MoShuffleboard.getInstance().intakeTab.addDouble("Roller Velocity (RPS)", () -> getIntakeVelocity()
                .in(Units.RotationsPerSecond));

        MoShuffleboard.getInstance().intakeTab.addDouble("Roller Current", () -> getRollerCurrent()
                .in(Units.Amps));

        MoShuffleboard.getInstance().intakeTab.add(this);
    }

    public void rollerIntake() {
        roller.setVoltage(Prefs.intakeRollerPower.get().in(Units.Volts));
    }

    public void rollerShoot() {
        roller.setVoltage(-1 * Prefs.intakeRollerPower.get().in(Units.Volts));
    }

    public void rollerProbe() {
        roller.setVoltage(-1 * Prefs.intakeRollerProbePower.get().in(Units.Volts));
    }

    public void holdBall() {
        roller.setVoltage(Prefs.intakeRollerHoldPower.get().in(Units.Volts));
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

    public Current getRollerCurrent() {
        return rollerCurrent.mut_replace(roller.getOutputCurrent(), Units.Amps);
    }

    @Override
    public void periodic() {
        rollerConfig.checkForBrownout();
    }
}
