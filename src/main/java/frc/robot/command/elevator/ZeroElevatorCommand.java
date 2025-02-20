// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO:
/*package frc.robot.command.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorMovementRequest;
import frc.robot.molib.prefs.MoPrefs;

public class ZeroElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private Timer currentTimer = new Timer();
    private boolean zeroed;

    private ElevatorMovementRequest getMovementRequest(double elevatorPower, double wristPower) {
        return new ElevatorMovementRequest(elevatorPower, wristPower);
    }

    public ZeroElevatorCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        currentTimer.stop();
        currentTimer.reset();
        zeroed = false;
    }

    @Override
    public void execute() {
        if (zeroed) {
            elevator.adjustDirectPower(getMovementRequest(0, 0));
            zeroed = true;

            return;
        }

        if (elevator.getElevatorCurrent() >= MoPrefs.elevatorZeroCurrentCutoff.get().in(Units.Amps)) {
            if (currentTimer.hasElapsed(MoPrefs.elevatorZeroTimeCutoff.get().in(Units.Seconds))) {
                elevator.setZero(Units.Centimeters.zero());
            }
        } else {
            currentTimer.restart();
        }

        elevator.adjustDirectPower(getMovementRequest(-Math.abs(MoPrefs.intakeZeroPwr.get()),0));
    }

    @Override
    public boolean isFinished() {
        return zeroed;
    }
}*/
