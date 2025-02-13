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
import frc.robot.command.EndEffectorExAlgaeInCoral;
import frc.robot.command.EndEffectorInAlgaeExCoral;
import frc.robot.command.TeleopDriveCommand;
import frc.robot.command.elevator.TeleopElevatorCommand;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.PositioningSubsystem;

public class RobotContainer {
    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private DriveSubsystem drive = new DriveSubsystem();
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro);
    private ElevatorSubsystem elevator = new ElevatorSubsystem();

    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);
    private TeleopElevatorCommand elevatorCommand = new TeleopElevatorCommand(elevator, this::getInput);
    private EndEffectorExAlgaeInCoral algaeOut = new EndEffectorExAlgaeInCoral(elevator);
    private EndEffectorInAlgaeExCoral algaeIn = new EndEffectorInAlgaeExCoral(elevator);

    private Trigger endEffectorExAlgaeInCoralTrigger;
    private Trigger endEffectorInAlgaeExCoralTrigger;

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        inputChooser.setDefaultOption("Single F310", new ControllerInput());

        drive.setDefaultCommand(driveCommand);
        elevator.setDefaultCommand(elevatorCommand);
    }

    private void configureBindings() {
        endEffectorExAlgaeInCoralTrigger = new Trigger(() -> getInput().getEndEffectorIn());
        endEffectorInAlgaeExCoralTrigger = new Trigger(() -> getInput().getEndEffectorOut());

        endEffectorExAlgaeInCoralTrigger.whileTrue(algaeOut);
        endEffectorInAlgaeExCoralTrigger.and(endEffectorExAlgaeInCoralTrigger.whileFalse(algaeIn));
    }

    private MoInput getInput() {
        return inputChooser.getSelected();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
