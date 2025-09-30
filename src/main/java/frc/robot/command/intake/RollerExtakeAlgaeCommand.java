package frc.robot.command.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Prefs;
import frc.robot.subsystem.IntakeRollerSubsystem;

public class RollerExtakeAlgaeCommand extends Command {
    private final IntakeRollerSubsystem roller;
    private final Timer timer;

    public RollerExtakeAlgaeCommand(IntakeRollerSubsystem roller) {
        this.roller = roller;
        this.timer = new Timer();

        addRequirements(roller);
    }

    @Override
    public void initialize() {
        this.timer.restart();
    }

    @Override
    public void execute() {
        this.roller.rollerProbe();
    }

    @Override
    public boolean isFinished() {
        return !this.roller.isBallHeld()
                || this.timer.hasElapsed(Prefs.intakeRollerExtakeTime.get().in(Units.Seconds));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Roller extake algae command finished");
        if (!interrupted) {
            this.roller.setBallHeld(false);
        }
    }
}
