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
import frc.robot.command.LEDsCommand;
import frc.robot.command.TeleopDriveCommand;
import frc.robot.command.climb.ClimberCommands;
import frc.robot.command.intake.IntakeCommands;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.subsystem.ClimberSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.IntakeRollerSubsystem;
import frc.robot.subsystem.IntakeWristSubsystem;
import frc.robot.subsystem.LEDsSubsystem;
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

    private Trigger intakeDeployTrigger;

    private Trigger extendClimberTrigger;
    private Trigger retractClimberTrigger;

    private final LEDsSubsystem ledsSubsystem = new LEDsSubsystem();

    private final LEDsCommand rainbowDefault = new LEDsCommand(ledsSubsystem, LEDsSubsystem.LEDMode.RAINBOW);
    private final LEDsCommand intakeColor = new LEDsCommand(ledsSubsystem, LEDsSubsystem.LEDMode.GROUND_INTAKE);
    private final LEDsCommand climberColor = new LEDsCommand(ledsSubsystem, LEDsSubsystem.LEDMode.CLIMBER);
    private final LEDsCommand endEffectorColor = new LEDsCommand(ledsSubsystem, LEDsSubsystem.LEDMode.END_EFFECTOR);
    private final LEDsCommand elevatorColor = new LEDsCommand(ledsSubsystem, LEDsSubsystem.LEDMode.ELEVATOR);

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

        endEffectorExAlgaeInCoralTrigger = new Trigger(() -> getInput().getEndEffectorIn());
        endEffectorInAlgaeExCoralTrigger = new Trigger(() -> getInput().getEndEffectorOut());

        sysidTrigger = new Trigger(() -> getInput().getRunSysid());

        intakeDeployTrigger.onTrue(teleopIntakeDeployCommand);
        intakeDeployTrigger.onFalse(teleopIntakeRetractCommand);

        extendClimberTrigger.whileTrue(ClimberCommands.extendClimber(climber, this::getInput));
        retractClimberTrigger.whileTrue(ClimberCommands.retractClimber(climber, this::getInput));

        endEffectorExAlgaeInCoralTrigger.whileTrue(algaeOutCommand);
        endEffectorInAlgaeExCoralTrigger.whileTrue(algaeInCommand);

        sysidTrigger.whileTrue(
                Commands.print("STARTING SYSID...").andThen(Commands.deferredProxy(sysidChooser::getSelected)));

        intakeDeployTrigger.onFalse(rainbowDefault);
        intakeDeployTrigger.onTrue(intakeColor);

        extendClimberTrigger.whileFalse(rainbowDefault);
        retractClimberTrigger.whileFalse(rainbowDefault);
        extendClimberTrigger.whileTrue(climberColor);
        retractClimberTrigger.whileTrue(climberColor);

        endEffectorExAlgaeInCoralTrigger.whileFalse(rainbowDefault);
        endEffectorInAlgaeExCoralTrigger.whileFalse(rainbowDefault);
        endEffectorExAlgaeInCoralTrigger.whiletrue(endEffectorColor);
        endEffectorInAlgaeExCoralTrigger.whiletrue(endEffectorColor);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getAutoCommand();
    }

    private MoInput getInput() {
        return inputChooser.getSelected();
    }
}
