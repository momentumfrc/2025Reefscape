package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;

public class GroundIntakeRollerSubsystem extends SubsystemBase {
    private final VictorSPX roller;
    private MutCurrent rollerCurrent = Units.Amps.mutable(0);

    public GroundIntakeRollerSubsystem() {
        super("Ground Intake Rollers");
        this.roller = new VictorSPX(Constants.INTAKE_ROLLER.address());
        this.roller.setNeutralMode(NeutralMode.Brake);

        MoShuffleboard.getInstance().groundIntakeTab.add(this);
    }

    public void rollerIntake() {
        roller.set(ControlMode.PercentOutput, MoPrefs.intakeRollerPercent.get().in(Units.Value));
    }

    public void rollerShoot() {
        roller.set(ControlMode.PercentOutput, -MoPrefs.intakeRollerPercent.get().in(Units.Value));
    }

    public void stopRollerMotor() {
        roller.set(ControlMode.PercentOutput, 0);
    }

    public Current getRollerCurrent() {
        double current = roller.getMotorOutputVoltage();
        rollerCurrent.mut_replace(current, Units.Amps);
        return rollerCurrent;
    }
}
