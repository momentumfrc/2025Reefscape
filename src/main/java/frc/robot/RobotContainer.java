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
import frc.robot.command.intake.HoldIntakeRollerCommand;
import frc.robot.command.intake.HoldIntakeWristCommand;
import frc.robot.command.intake.MoveIntakeRollersCommand;
import frc.robot.command.intake.MoveIntakeWristCommand;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.IntakeRollerSubsystem;
import frc.robot.subsystem.IntakeWristSubsystem;
import frc.robot.subsystem.PositioningSubsystem;

public class RobotContainer {
    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private DriveSubsystem drive = new DriveSubsystem();
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);

    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);

    private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
    private final IntakeWristSubsystem intakeWrist = new IntakeWristSubsystem();

    private final Command teleopIntakeDeployCommand = getGroundIntakeDeployCommand();
    private final Command teleopIntakeRetractCommand = getGroundIntakeRetractCommand();

    private final Command intakeRollersDefaultCommand = new HoldIntakeRollerCommand(intakeRoller);
    private final Command intakeWristDefaultCommand = new MoveIntakeWristCommand(
                    intakeWrist, MoveIntakeWristCommand.Direction.IN)
            .andThen(new HoldIntakeWristCommand(intakeWrist, HoldIntakeWristCommand.Direction.IN));

    private final Trigger intakeDeployTrigger;

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();

    public RobotContainer() {

        inputChooser.setDefaultOption("Single F310", new ControllerInput());

        drive.setDefaultCommand(driveCommand);
        intakeRoller.setDefaultCommand(intakeRollersDefaultCommand);
        intakeWrist.setDefaultCommand(intakeWristDefaultCommand);

        intakeDeployTrigger = new Trigger(() -> getInput().getIntake());
        configureBindings();
    }

    private void configureBindings() {
        intakeDeployTrigger.onTrue(teleopIntakeDeployCommand);
        intakeDeployTrigger.onFalse(teleopIntakeRetractCommand);
    }

    private Command getGroundIntakeDeployCommand() {
        return Commands.parallel(
                new MoveIntakeWristCommand(intakeWrist, MoveIntakeWristCommand.Direction.OUT)
                        .andThen(new HoldIntakeWristCommand(intakeWrist, HoldIntakeWristCommand.Direction.OUT)),
                new MoveIntakeRollersCommand(intakeRoller, MoveIntakeRollersCommand.Direction.INTAKE)
                        .andThen(new HoldIntakeRollerCommand(intakeRoller)));
    }

    private Command getGroundIntakeRetractCommand() {
        return Commands.parallel(
                        new MoveIntakeRollersCommand(intakeRoller, MoveIntakeRollersCommand.Direction.SHOOT),
                        new HoldIntakeWristCommand(intakeWrist, HoldIntakeWristCommand.Direction.OUT))
                .andThen(Commands.parallel(
                        new HoldIntakeRollerCommand(intakeRoller),
                        new MoveIntakeWristCommand(intakeWrist, MoveIntakeWristCommand.Direction.IN)
                                .andThen(
                                        new HoldIntakeWristCommand(intakeWrist, HoldIntakeWristCommand.Direction.IN))));
    }

    private MoInput getInput() {
        return inputChooser.getSelected();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
