package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;

public class EndEffectorSubsystem extends SubsystemBase {
    private final VictorSPX endEffector;

    public EndEffectorSubsystem() {
        super("End Effector");

        this.endEffector = new VictorSPX(Constants.END_EFFECTOR.address());

        MoShuffleboard.getInstance().elevatorTab.add(this);
    }

    public void intakeAlgaeCoralExtake() {
        endEffector.set(
                ControlMode.PercentOutput, MoPrefs.endEffectorPower.get().in(Units.Value));
    }

    public void extakeAlgaeCoralIntake() {
        endEffector.set(
                ControlMode.PercentOutput, -MoPrefs.endEffectorPower.get().in(Units.Value));
    }

    public void setEndEffector(double power) {
        endEffector.set(ControlMode.PercentOutput, power);
    }

    public void endEffectorStop() {
        endEffector.set(ControlMode.PercentOutput, 0);
    }
}
