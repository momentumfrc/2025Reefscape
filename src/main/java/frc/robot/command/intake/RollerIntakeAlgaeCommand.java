package frc.robot.command.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.IntakeRollerSubsystem;

public class RollerIntakeAlgaeCommand extends Command {
    enum State {
        SPINUP,
        SPINNING
    };

    private final IntakeRollerSubsystem roller;
    private final Timer timer = new Timer();
    private State state;

    public RollerIntakeAlgaeCommand(IntakeRollerSubsystem roller) {
        this.roller = roller;

        addRequirements(roller);
    }

    @Override
    public void initialize() {
        this.state = State.SPINUP;
        roller.setBallHeld(false);
        timer.restart();
    }

    @Override
    public void execute() {
        roller.rollerIntake();

        if (this.state == State.SPINUP) {
            if (timer.hasElapsed(MoPrefs.intakeRollerSpinupTime.get().in(Units.Seconds))) {
                timer.reset();
                this.state = State.SPINNING;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (this.state == State.SPINUP) {
            return false;
        }

        if (roller.getIntakeVelocity().lt(MoPrefs.intakeVelocityThreshold.get())) {
            if (this.timer.hasElapsed(MoPrefs.intakeRollerThesholdTime.get().in(Units.Seconds))) {
                return true;
            }
        } else {
            this.timer.reset();
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            roller.setBallHeld(true);
        }
    }
}
