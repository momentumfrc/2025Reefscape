package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.prefs.MoPrefs;
import edu.wpi.first.units.MutableMeasure;

public class IntakeSubsystem extends SubsystemBase {
    public static final String isDeployZeroed = null;
	private final SparkMax wrist;
    private final VictorSPX roller;

    //private final MutableMeasure<Current> intakeRollerCurrent = MutableMeasure.zero(Units.Amps);

    public IntakeSubsystem() {
        super("Intake");
        this.wrist = new SparkMax(Constants.INTAKE_WRIST.address(),MotorType.kBrushless);
        this.roller = new VictorSPX(Constants.INTAKE_ROLLER.address());
    }

    public void wristOut() {
        wrist.setVoltage(MoPrefs.intakeWristPower.get().in(Units.Volts));
    }

    public void wristIn() {
        wrist.setVoltage(-MoPrefs.intakeWristPower.get().in(Units.Volts));
    }

    public void rollerIntake() {
        roller.set(ControlMode.PercentOutput, MoPrefs.intakeRollerPower.get().in(Units.Volts));
    }

    public void rollerShoot() {
        roller.set(ControlMode.PercentOutput, -MoPrefs.intakeRollerPower.get().in(Units.Volts));
    }

    public void stopMotors() {
        roller.set(ControlMode.PercentOutput, 0);
        wrist.set(0);
    }

    /*public void intakeAlgae() {
        wristOut();
        rollerIntake();
    }

    public void shootAlgae() {
        wristIn();
        rollerShoot();
    } */
}
