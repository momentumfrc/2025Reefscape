package frc.robot.subsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;

public class GroundIntakeWristSubsystem extends SubsystemBase {
    private final SparkMax wrist;
    private MutCurrent wristCurrent = Units.Amps.mutable(0);

    public GroundIntakeWristSubsystem() {
        super("Ground Intake Wrist");
        this.wrist = new SparkMax(Constants.INTAKE_WRIST.address(), MotorType.kBrushless);

        MoShuffleboard.getInstance().groundIntakeTab.add(this);
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
