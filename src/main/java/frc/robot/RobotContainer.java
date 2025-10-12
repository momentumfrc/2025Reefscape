// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.momentum4999.molib.MoSparkConfigurator;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
    // private PowerDistribution pdh = new PowerDistribution();

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

    private Trigger rezeroElevatorTrigger;

    private Trigger extendClimberTrigger;
    private Trigger retractClimberTrigger;

    private Trigger wristInDangerTrigger;

    private Trigger climberSlowedTrigger;

    private Trigger sysidTrigger;

    private Trigger burnSparksTrigger;

    private final LEDsSubsystem ledsSubsystem = new LEDsSubsystem();

    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();
    private AutoChooser autoChooser = new AutoChooser(positioning, drive, elevator, endEffector);
    private SendableChooser<Command> sysidChooser = new SendableChooser<>();

    private final GenericEntry burnSparks =
            MoShuffleboard.getInstance().settingsTab.add("Burn sparks?", false).getEntry();

    private enum PidSubsystemToTune {
        NONE,
        ELEVATOR,
        WRIST;
    }

    private SendableChooser<PidSubsystemToTune> pidSubsystemChooser =
            MoShuffleboard.enumToChooser(PidSubsystemToTune.class);

    private final MoInput input;

    public RobotContainer() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        inputChooser.setDefaultOption("Joystick + F310", new ControllerInput());
        inputChooser.addOption("Dual Xbox Control", new DualXboxControllerInput());

        sysidChooser.setDefaultOption(
                "Elevator", MoShuffleboard.getInstance().getSysidCommand(elevator::getElevatorSysidMechanism));
        sysidChooser.addOption("Wrist", MoShuffleboard.getInstance().getSysidCommand(elevator::getWristSysidMechanism));

        MoShuffleboard.getInstance().settingsTab.add("Input", inputChooser);
        MoShuffleboard.getInstance().settingsTab.add("Sysid Mechanism", sysidChooser);
        MoShuffleboard.getInstance().settingsTab.add("Pid Subsystem to Tune", pidSubsystemChooser);

        // MoShuffleboard.getInstance()
        //         .elevatorTab
        //         .addDouble("End Effector Current", () -> pdh.getCurrent(Constants.END_EFFECTOR_ROLLERS_PDH_PORT));

        input = new MoInputTransforms(inputChooser::getSelected, this::getDriveSlewRate, this::getMaxThrottle);

        configureBindings();
        setDefaultCommands();
    }

    public void setDefaultCommands() {
        drive.setDefaultCommand(driveCommand);
        climber.setDefaultCommand(ClimberCommands.idleClimber(climber));
        intakeRoller.setDefaultCommand(intakeRollersDefaultCommand);
        intakeWrist.setDefaultCommand(intakeWristDefaultCommand);
        elevator.setDefaultCommand(elevatorCommand);
        endEffector.setDefaultCommand(endEffectorIdle);
        ledsSubsystem.setDefaultCommand(LEDCommands.defaultPattern(ledsSubsystem));
    }

    public void setDefaultCommandsForAuto() {
        drive.setDefaultCommand(
                Commands.run(() -> drive.driveCartesian(0, 0, 0), drive).withName("IdleDriveCommand"));
        elevator.setDefaultCommand(Commands.run(
                        () -> elevator.adjustVelocity(new ElevatorSubsystem.ElevatorMovementRequest(0, 0)), elevator)
                .withName("IdleElevatorCommand"));
    }

    private void configureBindings() {
        intakeDeployTrigger = new Trigger(() -> getInput().getIntake());

        extendClimberTrigger = new Trigger(() -> getInput().getClimberMoveRequest() > 0);
        retractClimberTrigger = new Trigger(() -> getInput().getClimberMoveRequest() < 0);

        rezeroElevatorTrigger = new Trigger(() -> getInput().getReZeroElevator());

        endEffectorExAlgaeInCoralTrigger = new Trigger(() -> getInput().getEndEffectorIn());
        endEffectorInAlgaeExCoralTrigger = new Trigger(() -> getInput().getEndEffectorOut());

        wristInDangerTrigger = new Trigger(() -> elevator.isWristInDanger());

        sysidTrigger = new Trigger(() -> getInput().getRunSysid());

        intakeExtakeOverrideTrigger = new Trigger(() -> getInput().getIntakeExtakeOverride());

        burnSparksTrigger = new Trigger(() -> burnSparks.getBoolean(false));

        climberSlowedTrigger = new Trigger(() -> getMaxThrottle() < 0.95);

        var teleopTrigger = RobotModeTriggers.teleop();

        intakeDeployTrigger.and(teleopTrigger).onTrue(teleopIntakeDeployCommand);
        intakeExtakeOverrideTrigger
                .and(teleopTrigger)
                .onTrue(IntakeCommands.intakeExtakeOverrideCommand(intakeWrist, intakeRoller));
        intakeDeployTrigger.or(intakeExtakeOverrideTrigger).onFalse(teleopIntakeRetractCommand);

        extendClimberTrigger.and(teleopTrigger).whileTrue(ClimberCommands.extendClimber(climber, this::getInput));
        retractClimberTrigger.and(teleopTrigger).whileTrue(ClimberCommands.retractClimber(climber, this::getInput));
        endEffectorExAlgaeInCoralTrigger.and(teleopTrigger).whileTrue(algaeOutCommand);
        endEffectorInAlgaeExCoralTrigger.and(teleopTrigger).whileTrue(algaeInCommand);

        intakeDeployTrigger.whileTrue(LEDCommands.groundIntakePattern(ledsSubsystem));
        climberSlowedTrigger.whileTrue(LEDCommands.climberSlowedPattern(ledsSubsystem));

        wristInDangerTrigger.whileTrue(LEDCommands.wristInDangerPattern(ledsSubsystem));

        sysidTrigger.whileTrue(
                Commands.print("STARTING SYSID...").andThen(Commands.deferredProxy(sysidChooser::getSelected)));

        burnSparksTrigger.onTrue(
                Commands.runOnce(MoSparkConfigurator::persistAllParameters).ignoringDisable(true));

        rezeroElevatorTrigger
                .and(RobotModeTriggers.teleop())
                .whileTrue(Commands.runOnce(() -> elevator.setElevatorHasZero(false), elevator)
                        .andThen(new ZeroElevatorCommand(elevator))
                        .finallyDo(interrupted -> {
                            if (interrupted) {
                                elevator.setElevatorHasZero(true);
                            }
                        }));
    }

    private double getDriveSlewRate() {
        double elevatorExtensionPercent = elevator.getElevatorHeight().in(Units.Centimeters)
                / Prefs.elevatorMaxExtension.get().in(Units.Centimeters);
        double rampTime = MathUtil.interpolate(
                Prefs.driveRampTime.get().in(Units.Seconds),
                Prefs.driveRampTimeElevatorExtended.get().in(Units.Seconds),
                elevatorExtensionPercent);
        return 1.0 / rampTime;
    }

    private double getMaxThrottle() {
        if (!climber.isZeroed()) {
            return 1;
        }

        double climberRetractedZone = Prefs.climberRetractedZone.get();
        double climberPos = (climber.getClimberPosition() - climberRetractedZone)
                / (Prefs.climberFwdSoftLimit.get() - climberRetractedZone);
        return MathUtil.interpolate(
                1, Prefs.driveMaxThrottleClimberExtended.get().in(Units.Value), climberPos);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getAutoCommand();
    }

    private MoInput getInput() {
        return input;
    }
}
