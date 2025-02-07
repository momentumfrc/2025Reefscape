package frc.robot.command.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.input.MoInput;
import frc.robot.subsystem.ClimberSubsystem;
import frc.robot.subsystem.ClimberSubsystem.RachetState;
import java.util.function.Supplier;

public class ClimberCommands {

    public static Command idleClimber(ClimberSubsystem climber) {
        return new MoveRachetCommand(climber, RachetState.ENGAGED)
                .andThen(Commands.run(climber::idleClimber, climber))
                .withName("IdleCommand");
    }

    public static Command extendClimber(ClimberSubsystem climber, Supplier<MoInput> inputSupplier) {
        return new MoveRachetCommand(climber, RachetState.DISENGAGED)
                .andThen(Commands.run(
                        () -> climber.extendClimber(inputSupplier.get().getClimberMoveRequest()), climber))
                .withName("ExtendCommand");
    }

    public static Command retractClimber(ClimberSubsystem climber, Supplier<MoInput> inputSupplier) {
        return new MoveRachetCommand(climber, RachetState.ENGAGED)
                .andThen(Commands.run(
                        () -> climber.retractClimber(inputSupplier.get().getClimberMoveRequest()), climber))
                .withName("RetractCommand");
    }

    private ClimberCommands() {
        throw new UnsupportedOperationException("ClimberCommands is a static class");
    }
}
