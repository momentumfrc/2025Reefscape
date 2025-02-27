// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command.EndEffectorCommands;
import frc.robot.command.LEDCommands;
import frc.robot.command.TeleopDriveCommand;
import frc.robot.command.climb.ClimberCommands;
import frc.robot.command.elevator.TeleopElevatorCommand;
import frc.robot.command.elevator.ZeroElevatorCommand;
import frc.robot.command.intake.IntakeCommands;
import frc.robot.input.ControllerInput;
import frc.robot.input.DualXboxControllerInput;
import frc.robot.input.MoInput;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.ClimberSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.EndEffectorSubsystem;
import frc.robot.subsystem.IntakeRollerSubsystem;
import frc.robot.subsystem.IntakeWristSubsystem;
import frc.robot.subsystem.LEDsSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.utils.AutoChooser;
import frc.robot.utils.MoInputTransforms;

public class RobotContainer {
    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private DriveSubsystem drive = new DriveSubsystem();
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);
    private ClimberSubsystem climber = new ClimberSubsystem();
    private ElevatorSubsystem elevator = new ElevatorSubsystem();
    private EndEffectorSubsystem endEffector = new EndEffectorSubsystem();

    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);
    private final Command elevatorCommand = new ZeroElevatorCommand(elevator)
            .andThen(new TeleopElevatorCommand(elevator, this::getInput))
            .withName("ZeroThenTeleopElevatorCommand");
    private final Command algaeOutCommand = EndEffectorCommands.exAlgaeInCoral(endEffector);
    private final Command algaeInCommand = EndEffectorCommands.inAlgaeExCoral(endEffector);
    private final Command endEffectorIdle = EndEffectorCommands.idleEndEffector(endEffector);

    private Trigger endEffectorExAlgaeInCoralTrigger;
    private Trigger endEffectorInAlgaeExCoralTrigger;

    private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
    private final IntakeWristSubsystem intakeWrist = new IntakeWristSubsystem();

    private final Command teleopIntakeDeployCommand = IntakeCommands.intakeDeployCommand(intakeWrist, intakeRoller);
    private final Command teleopIntakeRetractCommand = IntakeCommands.intakeRetractCommand(intakeWrist, intakeRoller);

    private final Command intakeRollersDefaultCommand = IntakeCommands.intakeRollerDefaultCommand(intakeRoller);
    private final Command intakeWristDefaultCommand = IntakeCommands.intakeWristDefaultCommand(intakeWrist);

    private Trigger intakeDeployTrigger;
    private Trigger intakeExtakeOverrideTrigger;

    private Trigger extendClimberTrigger;
    private Trigger retractClimberTrigger;

    private Trigger raiseElevatorTrigger;
    private Trigger lowerElevatorTrigger;

    private Trigger wristInDangerTrigger;

    private Trigger sysidTrigger;

    private final LEDsSubsystem ledsSubsystem = new LEDsSubsystem();

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();
    private AutoChooser autoChooser = new AutoChooser(positioning, drive, elevator, endEffector);
    private SendableChooser<Command> sysidChooser = new SendableChooser<>();

    private enum PidSubsystemToTune {
        NONE,
        ELEVATOR,
        WRIST;
    }

    private SendableChooser<PidSubsystemToTune> pidSubsystemChooser =
            MoShuffleboard.enumToChooser(PidSubsystemToTune.class);

    private final MoInput input;

    public RobotContainer() {
        inputChooser.setDefaultOption("Joystick + F310", new ControllerInput());
        inputChooser.addOption("Dual Xbox Control", new DualXboxControllerInput());

        sysidChooser.setDefaultOption(
                "Elevator", MoShuffleboard.getInstance().getSysidCommand(elevator::getElevatorSysidMechanism));
        sysidChooser.addOption("Wrist", MoShuffleboard.getInstance().getSysidCommand(elevator::getWristSysidMechanism));

        MoShuffleboard.getInstance().settingsTab.add("Input", inputChooser);
        MoShuffleboard.getInstance().settingsTab.add("Sysid Mechanism", sysidChooser);
        MoShuffleboard.getInstance().settingsTab.add("Pid Subsystem to Tune", pidSubsystemChooser);

        input = new MoInputTransforms(inputChooser::getSelected, this::getDriveSlewRate);

        configureBindings();

        drive.setDefaultCommand(driveCommand);
        climber.setDefaultCommand(ClimberCommands.idleClimber(climber));
        intakeRoller.setDefaultCommand(intakeRollersDefaultCommand);
        intakeWrist.setDefaultCommand(intakeWristDefaultCommand);
        elevator.setDefaultCommand(elevatorCommand);
        endEffector.setDefaultCommand(endEffectorIdle);
        ledsSubsystem.setDefaultCommand(LEDCommands.defaultPattern(ledsSubsystem));
    }

    private void configureBindings() {
        intakeDeployTrigger = new Trigger(() -> getInput().getIntake());

        extendClimberTrigger = new Trigger(() -> getInput().getClimberMoveRequest() > 0);
        retractClimberTrigger = new Trigger(() -> getInput().getClimberMoveRequest() < 0);

        endEffectorExAlgaeInCoralTrigger = new Trigger(() -> getInput().getEndEffectorIn());
        endEffectorInAlgaeExCoralTrigger = new Trigger(() -> getInput().getEndEffectorOut());

        raiseElevatorTrigger =
                new Trigger(() -> getInput().getElevatorMovementRequest().elevatorPower() > 0);
        lowerElevatorTrigger =
                new Trigger(() -> getInput().getElevatorMovementRequest().elevatorPower() < 0);

        wristInDangerTrigger = new Trigger(() -> elevator.isWristInDanger() && DriverStation.isEnabled());

        sysidTrigger = new Trigger(() -> getInput().getRunSysid());

        intakeExtakeOverrideTrigger = new Trigger(() -> getInput().getIntakeExtakeOverride());

        intakeDeployTrigger.onTrue(teleopIntakeDeployCommand);
        intakeExtakeOverrideTrigger.onTrue(IntakeCommands.intakeExtakeOverrideCommand(intakeWrist, intakeRoller));
        intakeDeployTrigger.or(intakeExtakeOverrideTrigger).onFalse(teleopIntakeRetractCommand);

        extendClimberTrigger.whileTrue(ClimberCommands.extendClimber(climber, this::getInput));
        retractClimberTrigger.whileTrue(ClimberCommands.retractClimber(climber, this::getInput));
        endEffectorExAlgaeInCoralTrigger.whileTrue(algaeOutCommand);
        endEffectorInAlgaeExCoralTrigger.whileTrue(algaeInCommand);

        intakeDeployTrigger.whileTrue(LEDCommands.groundIntakePattern(ledsSubsystem));

        retractClimberTrigger.or(extendClimberTrigger).whileTrue(LEDCommands.climberPattern(ledsSubsystem));

        endEffectorInAlgaeExCoralTrigger
                .or(endEffectorExAlgaeInCoralTrigger)
                .whileTrue(LEDCommands.endEffectorPattern(ledsSubsystem));

        raiseElevatorTrigger.or(lowerElevatorTrigger).whileTrue(LEDCommands.elevatorPattern(ledsSubsystem));

        wristInDangerTrigger.whileTrue(LEDCommands.wristInDangerPattern(ledsSubsystem));

        sysidTrigger.whileTrue(
                Commands.print("STARTING SYSID...").andThen(Commands.deferredProxy(sysidChooser::getSelected)));
    }

    private double getDriveSlewRate() {
        double elevatorExtensionPercent = elevator.getElevatorHeight().in(Units.Centimeters)
                / MoPrefs.elevatorMaxExtension.get().in(Units.Centimeters);
        double rampTime = MathUtil.interpolate(
                MoPrefs.driveRampTime.get().in(Units.Seconds),
                MoPrefs.driveRampTimeElevatorExtended.get().in(Units.Seconds),
                elevatorExtensionPercent);
        return 1.0 / rampTime;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getAutoCommand();
    }

    private MoInput getInput() {
        return input;
    }
}
