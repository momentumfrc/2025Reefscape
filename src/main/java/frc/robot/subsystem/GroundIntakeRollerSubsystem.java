package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;

public class GroundIntakeRollerSubsystem extends SubsystemBase {
    private final VictorSPX roller;

    // private final MutableMeasure<Current> intakeRollerCurrent = MutableMeasure.zero(Units.Amps);

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

    /*public void intakeAlgae() {
        wristOut();
        rollerIntake();
    }

    public void shootAlgae() {
        wristIn();
        rollerShoot();
    } */
}
