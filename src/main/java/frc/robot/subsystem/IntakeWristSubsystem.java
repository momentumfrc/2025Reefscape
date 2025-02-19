package frc.robot.subsystem;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;

public class IntakeWristSubsystem extends SubsystemBase {
    private static final Current SMART_CURRENT_LIMIT = Units.Amps.of(20);

    private final SparkMax wrist;
    private MutCurrent wristCurrent = Units.Amps.mutable(0);

    public IntakeWristSubsystem() {
        super("Ground Intake Wrist");
        this.wrist = new SparkMax(Constants.INTAKE_WRIST.address(), MotorType.kBrushless);

        this.wrist.configure(
                new SparkMaxConfig()
                        .smartCurrentLimit((int) SMART_CURRENT_LIMIT.in(Units.Amps))
                        .idleMode(IdleMode.kCoast),
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        MoShuffleboard.getInstance().intakeTab.addDouble("Wrist Current", () -> getWristCurrent()
                .in(Units.Amps));
        MoShuffleboard.getInstance().intakeTab.add(this);
    }

    public void wristOut() {
        wrist.setVoltage(MoPrefs.intakeWristPower.get().in(Units.Volts));
    }

    public void wristIn() {
        wrist.setVoltage(-MoPrefs.intakeWristPower.get().in(Units.Volts));
    }

    public void holdWristOut() {
        wrist.setVoltage(MoPrefs.intakeWristHoldPower.get().in(Units.Volts));
    }

    public void holdWristIn() {
        wrist.setVoltage(-MoPrefs.intakeWristHoldPower.get().in(Units.Volts));
    }

    public void stopWristMotors() {
        wrist.set(0);
    }

    public Current getWristCurrent() {
        double current = wrist.getOutputCurrent();
        wristCurrent.mut_replace(current, Units.Amps);
        return wristCurrent;
    }
}
