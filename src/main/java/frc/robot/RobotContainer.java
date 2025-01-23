// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.command.TeleopDriveCommand;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;

public class RobotContainer {
    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private DriveSubsystem drive = new DriveSubsystem();
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);

    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        inputChooser.setDefaultOption("Single F310", new ControllerInput());

        drive.setDefaultCommand(driveCommand);
    }

    public Command getAutonomousCommand() {
        // Load the path you want to follow using its name in the GUI
        return PathPlannerCommands.getFollowPathCommand(drive, positioning, "Example Path", false);
    }

    private void configureBindings() {}

    private MoInput getInput() {
        return inputChooser.getSelected();
    }

}
