package frc.robot.command.climb;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.ClimberSubsystem;

public class ZeroEncodersCommand extends Command {
    private final ClimberSubsystem climber;

    public ZeroEncodersCommand(ClimberSubsystem climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.disableReverseLimit();
    }

    @Override
    public void execute() {
        climber.retractClimber(-MoPrefs.climberZeroPwr.get().in(Units.Value));

        if (climber.isLimitSwitchPressed()) {
            climber.rezeroEncoder();
        }
    }

    @Override
    public boolean isFinished() {
        return climber.isZeroed();
    }
}
