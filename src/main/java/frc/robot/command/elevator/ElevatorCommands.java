package frc.robot.command.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.component.ElevatorSetpointManager;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorPosition;

public class ElevatorCommands {
    public static Command waitForSetpoint(ElevatorSubsystem elevator, ElevatorSetpoint setpoint) {
        return Commands.waitUntil(() -> elevator.atPosition(
                        ElevatorSetpointManager.getInstance().getSetpoint(setpoint)))
                .withName(String.format("WaitForElevatorSetpoint(%s)", setpoint.name()));
    }

    public static Command holdSetpoint(ElevatorSubsystem elevator, ElevatorSetpoint setpoint) {
        return Commands.run(
                        () -> elevator.adjustPosition(
                                ElevatorSetpointManager.getInstance().getSetpoint(setpoint)),
                        elevator)
                .withName(String.format("HoldElevatorSetpoint(%s)", setpoint.name()));
    }

    public static Command moveToSetpoint(ElevatorSubsystem elevator, ElevatorSetpoint setpoint) {
        return Commands.deadline(waitForSetpoint(elevator, setpoint), holdSetpoint(elevator, setpoint))
                .withName(String.format("MoveToElevatorSetpoint(%s)", setpoint.name()));
    }

    public static Command getTuneElevatorCommand(ElevatorSubsystem elevator) {
        MutDistance setpoint = Units.Centimeters.mutable(0);
        return Commands.run(
                () -> {
                    ElevatorPosition stowPos =
                            ElevatorSetpointManager.getInstance().getSetpoint(ElevatorSetpoint.STOW);
                    setpoint.mut_replace(
                            Math.sin(WPIUtilJNI.getSystemTime() / 1_000_000.0) * 5 + 10, Units.Centimeters);
                    ElevatorPosition pos = new ElevatorPosition(setpoint, stowPos.wristAngle());
                    elevator.adjustPosition(pos);
                },
                elevator);
    }

    public static Command getTuneWristCommand(ElevatorSubsystem elevator) {
        MutAngle setpoint = Units.Rotations.mutable(0);
        return Commands.run(
                () -> {
                    ElevatorPosition stowPos =
                            ElevatorSetpointManager.getInstance().getSetpoint(ElevatorSetpoint.STOW);
                    setpoint.mut_replace(
                            Math.sin(WPIUtilJNI.getSystemTime() / 1_000_000.0) * 0.1 + 0.1, Units.Rotations);
                    ElevatorPosition pos = new ElevatorPosition(stowPos.elevatorDistance(), setpoint);
                    elevator.adjustPosition(pos);
                },
                elevator);
    }

    private ElevatorCommands() {
        throw new UnsupportedOperationException("ElevatorCommands is a static class");
    }
}
