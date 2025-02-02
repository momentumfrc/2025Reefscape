// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command.TeleopDriveCommand;
import frc.robot.command.climb.ClimberCommands;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.subsystem.ClimberSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;

public class RobotContainer {
    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private DriveSubsystem drive = new DriveSubsystem();
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);
    private ClimberSubsystem climber = new ClimberSubsystem();

    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();

    private Trigger extendClimberTrigger;
    private Trigger retractClimberTrigger;

    public RobotContainer() {
        inputChooser.setDefaultOption("Single F310", new ControllerInput());

        configureBindings();

        drive.setDefaultCommand(driveCommand);
        climber.setDefaultCommand(ClimberCommands.idleClimber(climber));
    }

    private void configureBindings() {
        extendClimberTrigger = new Trigger(() -> getInput().getClimberMoveRequest() > 0);
        retractClimberTrigger = new Trigger(() -> getInput().getClimberMoveRequest() < 0);

        extendClimberTrigger.whileTrue(ClimberCommands.extendClimber(climber, this::getInput));
        retractClimberTrigger.whileTrue(ClimberCommands.retractClimber(climber, this::getInput));
    }

    private MoInput getInput() {
        return inputChooser.getSelected();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
