// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command.AutoAlignCommand;
import frc.robot.command.AutoAlignCommand.AutoAlignMode;
import frc.robot.command.TeleopDriveCommand;
import frc.robot.command.climb.ClimberCommands;
import frc.robot.command.intake.IntakeCommands;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.subsystem.ClimberSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.IntakeRollerSubsystem;
import frc.robot.subsystem.IntakeWristSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.utils.AutoChooser;

public class RobotContainer {
    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private DriveSubsystem drive = new DriveSubsystem();
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);
    private ClimberSubsystem climber = new ClimberSubsystem();

    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);

    private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
    private final IntakeWristSubsystem intakeWrist = new IntakeWristSubsystem();

    private final Command teleopIntakeDeployCommand = IntakeCommands.intakeDeployCommand(intakeWrist, intakeRoller);
    private final Command teleopIntakeRetractCommand = IntakeCommands.intakeRetractCommand(intakeWrist, intakeRoller);

    private final Command intakeRollersDefaultCommand = IntakeCommands.intakeRollerDefaultCommand(intakeRoller);
    private final Command intakeWristDefaultCommand = IntakeCommands.intakeWristDefaultCommand(intakeWrist);

    private final Command autoAlignCoralLeftCommand =
            new AutoAlignCommand(drive, positioning, AutoAlignMode.CORAL_LEFT);
    private final Command autoAlignCoralRightCommand =
            new AutoAlignCommand(drive, positioning, AutoAlignMode.CORAL_RIGHT);
    private final Command autoAlignAlgaeMiddleCommand =
            new AutoAlignCommand(drive, positioning, AutoAlignMode.ALGAE_MIDDLE);

    private Trigger intakeDeployTrigger;

    private Trigger extendClimberTrigger;
    private Trigger retractClimberTrigger;

    private Trigger autoAlignCoralLeftTrigger;
    private Trigger autoAlignCoralRightTrigger;
    private Trigger autoAlignAlgaeMiddleTrigger;

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();
    private AutoChooser autoChooser = new AutoChooser(positioning, drive);

    public RobotContainer() {
        inputChooser.setDefaultOption("Single F310", new ControllerInput());

        configureBindings();

        drive.setDefaultCommand(driveCommand);
        climber.setDefaultCommand(ClimberCommands.idleClimber(climber));
        intakeRoller.setDefaultCommand(intakeRollersDefaultCommand);
        intakeWrist.setDefaultCommand(intakeWristDefaultCommand);
    }

    private void configureBindings() {
        intakeDeployTrigger = new Trigger(() -> getInput().getIntake());

        extendClimberTrigger = new Trigger(() -> getInput().getClimberMoveRequest() > 0);
        retractClimberTrigger = new Trigger(() -> getInput().getClimberMoveRequest() < 0);

        autoAlignCoralLeftTrigger = new Trigger(() -> getInput().getAutoAlignCoralLeft());
        autoAlignCoralRightTrigger = new Trigger(() -> getInput().getAutoAlignCoralRight());
        autoAlignAlgaeMiddleTrigger = new Trigger(() -> getInput().getAutoAlignAlgaeMiddle());

        intakeDeployTrigger.onTrue(teleopIntakeDeployCommand);
        intakeDeployTrigger.onFalse(teleopIntakeRetractCommand);

        extendClimberTrigger.whileTrue(ClimberCommands.extendClimber(climber, this::getInput));
        retractClimberTrigger.whileTrue(ClimberCommands.retractClimber(climber, this::getInput));

        autoAlignCoralLeftTrigger.whileTrue(autoAlignCoralLeftCommand);
        autoAlignCoralRightTrigger.whileTrue(autoAlignCoralRightCommand);
        autoAlignAlgaeMiddleTrigger.whileTrue(autoAlignAlgaeMiddleCommand);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getAutoCommand();
    }

    private MoInput getInput() {
        return inputChooser.getSelected();
    }
}
