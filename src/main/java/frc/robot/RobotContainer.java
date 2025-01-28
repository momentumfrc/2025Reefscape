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
import frc.robot.command.groundintake.HoldGroundIntakeRollerCommand;
import frc.robot.command.groundintake.HoldGroundIntakeWristCommand;
import frc.robot.command.groundintake.MoveGroundIntakeRollersCommand;
import frc.robot.command.groundintake.MoveGroundWristCommand;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.GroundIntakeRollerSubsystem;
import frc.robot.subsystem.GroundIntakeWristSubsystem;
import frc.robot.subsystem.PositioningSubsystem;

public class RobotContainer {
    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private DriveSubsystem drive = new DriveSubsystem();
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);

    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);

    private final GroundIntakeRollerSubsystem intakeRoller = new GroundIntakeRollerSubsystem();
    private final GroundIntakeWristSubsystem intakeWrist = new GroundIntakeWristSubsystem();

    private final Command teleopGroundIntakeDeployCommand = getGroundIntakeDeployCommand();
    private final Command teleopGroundIntakeRetractCommand = getGroundIntakeRetractCommand();

    private final Command groundIntakeRollersDefaultCommand = new HoldGroundIntakeRollerCommand(intakeRoller);
    private final Command groundIntakeWristDefaultCommand = new MoveGroundWristCommand(
                    intakeWrist, MoveGroundWristCommand.Direction.IN)
            .andThen(new HoldGroundIntakeWristCommand(intakeWrist, HoldGroundIntakeWristCommand.Direction.IN));

    private final Trigger intakeDeployTrigger;

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();

    public RobotContainer() {

        inputChooser.setDefaultOption("Single F310", new ControllerInput());

        drive.setDefaultCommand(driveCommand);
        intakeRoller.setDefaultCommand(groundIntakeRollersDefaultCommand);
        intakeWrist.setDefaultCommand(groundIntakeWristDefaultCommand);

        intakeDeployTrigger = new Trigger(() -> getInput().getIntake());
        configureBindings();
    }

    private void configureBindings() {
        intakeDeployTrigger.onTrue(teleopGroundIntakeDeployCommand);
        intakeDeployTrigger.onFalse(teleopGroundIntakeRetractCommand);
    }

    private Command getGroundIntakeDeployCommand() {
        return Commands.parallel(
                new MoveGroundWristCommand(intakeWrist, MoveGroundWristCommand.Direction.OUT)
                        .andThen(new HoldGroundIntakeWristCommand(
                                intakeWrist, HoldGroundIntakeWristCommand.Direction.OUT)),
                new MoveGroundIntakeRollersCommand(intakeRoller, MoveGroundIntakeRollersCommand.Direction.INTAKE)
                        .andThen(new HoldGroundIntakeRollerCommand(intakeRoller)));
    }

    private Command getGroundIntakeRetractCommand() {
        return Commands.parallel(
                        new MoveGroundIntakeRollersCommand(
                                intakeRoller, MoveGroundIntakeRollersCommand.Direction.SHOOT),
                        new HoldGroundIntakeWristCommand(intakeWrist, HoldGroundIntakeWristCommand.Direction.OUT))
                .andThen(Commands.parallel(
                        new HoldGroundIntakeRollerCommand(intakeRoller),
                        new MoveGroundWristCommand(intakeWrist, MoveGroundWristCommand.Direction.IN)
                                .andThen(new HoldGroundIntakeWristCommand(
                                        intakeWrist, HoldGroundIntakeWristCommand.Direction.IN))));
    }

    private MoInput getInput() {
        return inputChooser.getSelected();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
