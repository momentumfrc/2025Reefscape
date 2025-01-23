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
import frc.robot.command.TeleopIntakeCommand;
import frc.robot.command.TeleopShootCommand;
import frc.robot.command.TeleopWristInCommand;
import frc.robot.command.TeleopWristOutCommand;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.PositioningSubsystem;

public class RobotContainer {
    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private DriveSubsystem drive = new DriveSubsystem();
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro);

    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);

    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final TeleopIntakeCommand intakeCommand = new TeleopIntakeCommand(intake, this::getInput);
    private final TeleopShootCommand shootCommand = new TeleopShootCommand(intake, this::getInput);
    private final TeleopWristOutCommand wristOutCommand = new TeleopWristOutCommand(intake, this::getInput);
    private final TeleopWristInCommand wristInCommand = new TeleopWristInCommand(intake, this::getInput);
    private final Trigger intakeDeployTrigger;
    private final Trigger intakRetractTrigger;
    private final Trigger intakeAlgaeTrigger;
    private final Trigger intakeShootTrigger;

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        inputChooser.setDefaultOption("Single F310", new ControllerInput());

        drive.setDefaultCommand(driveCommand);

        intakeDeployTrigger = new Trigger(() -> getInput().getIntakeOut());
        intakRetractTrigger = new Trigger(() -> getInput().getIntakeIn());
        intakeAlgaeTrigger = new Trigger(() -> getInput().getIntakeAlgae());
        intakeShootTrigger = new Trigger(() -> getInput().getIntakeShoot());
    }

    private void configureBindings() {
        intakeDeployTrigger.whileTrue(wristOutCommand);
        intakRetractTrigger.whileTrue(wristInCommand);
        intakeAlgaeTrigger.whileTrue(intakeCommand);
        intakeShootTrigger.whileTrue(shootCommand);
    }

    private MoInput getInput() {
        return inputChooser.getSelected();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
