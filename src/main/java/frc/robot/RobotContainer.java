// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;

public class RobotContainer {

    private DriveSubsystem drive = new DriveSubsystem();

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        inputChooser.setDefaultOption("Single F310", new ControllerInput());
    }

    private void configureBindings() {}

    private MoInput getInput() {
        return inputChooser.getSelected();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
