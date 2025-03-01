package frc.robot.utils;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.command.elevator.ElevatorCommands;
import frc.robot.command.elevator.ZeroElevatorCommand;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.EndEffectorSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import java.util.List;

public class AutoChooser {
    private enum AutoChoices {
        DYNAMIC_LEAVE,
        INITIAL_POSITION_LEAVE,
        FALLBACK_LEAVE
    }

    private enum InitialPosition {
        BLUE_WALL(new Pose2d(7.5, 7.5, Rotation2d.k180deg)),
        RED_WALL(new Pose2d(7.5, 0.5, Rotation2d.k180deg));

        public final Pose2d pose;

        private InitialPosition(Pose2d pose) {
            this.pose = pose;
        }
    }

    private final GenericEntry masterAutoSwitch;
    private final SendableChooser<AutoChoices> autoChoicesChooser = MoShuffleboard.enumToChooser(AutoChoices.class);
    private final SendableChooser<InitialPosition> initialPositionChooser =
            MoShuffleboard.enumToChooser(InitialPosition.class);
    private final GenericEntry shouldScoreL1Coral;

    private final PositioningSubsystem positioning;
    private final DriveSubsystem drive;
    private final ElevatorSubsystem elevator;
    private final EndEffectorSubsystem endEffector;

    public AutoChooser(
            PositioningSubsystem positioning,
            DriveSubsystem drive,
            ElevatorSubsystem elevator,
            EndEffectorSubsystem endEffector) {
        this.positioning = positioning;
        this.drive = drive;
        this.elevator = elevator;
        this.endEffector = endEffector;

        masterAutoSwitch = MoShuffleboard.getInstance()
                .autoTab
                .add("Enable Auto?", true)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        MoShuffleboard.getInstance().autoTab.add("Auto Choices", autoChoicesChooser);
        MoShuffleboard.getInstance().autoTab.add("Initial Position", initialPositionChooser);

        shouldScoreL1Coral = MoShuffleboard.getInstance()
                .autoTab
                .add("Score L1 preload?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();
    }

    private Command leaveFromPose(Pose2d initialPos, boolean assumeRobotAtPos) {
        // Don't need to flip the tranform because the transform is in the direction of the pose, and the pose
        // is already flipped
        Transform2d transform = new Transform2d(MoPrefs.autoLeaveDist.get().in(Units.Meters), 0, Rotation2d.kZero);
        Pose2d destPos = initialPos.plus(transform);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(initialPos, destPos);
        PathConstraints constraints = new PathConstraints(
                MoPrefs.autoMaxLinVel.get(),
                MoPrefs.autoMaxLinAccel.get(),
                MoPrefs.autoMaxAngVel.get(),
                MoPrefs.autoMaxAngAccel.get());
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(0, initialPos.getRotation()),
                new GoalEndState(0, initialPos.getRotation()));
        path.preventFlipping = true;

        return PathPlannerCommands.getFollowPathCommand(drive, positioning, path, assumeRobotAtPos);
    }

    private Command dynamicLeave() {
        if (!positioning.hasInitialPosition()) {
            return Commands.print("No initial position established, using fallback autonomous.")
                    .andThen(fallbackLeave());
        }

        return leaveFromPose(positioning.getRobotPose(), false);
    }

    private Command leaveFromInitialPosition(InitialPosition pos) {
        Pose2d pose = pos.pose;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            pose = new Pose2d(
                    new Translation2d(
                            FlippingUtil.fieldSizeX - pose.getTranslation().getX(),
                            pose.getTranslation().getY()),
                    pose.getRotation().plus(Rotation2d.k180deg));
        }

        return leaveFromPose(pose, true);
    }

    private Command fallbackLeave() {
        return Commands.run(
                        () -> drive.driveCartesian(MoPrefs.autoFallbackSpd.get().in(Units.Value), 0, 0), drive)
                .withTimeout(MoPrefs.autoFallbackTime.get())
                .withName("FallbackLeaveCommand");
    }

    public Command getAutoCommand() {
        if (!masterAutoSwitch.getBoolean(true)) {
            return Commands.print("Autonomous disabled by master switch");
        }

        var auto =
                switch (autoChoicesChooser.getSelected()) {
                    case DYNAMIC_LEAVE -> dynamicLeave();
                    case INITIAL_POSITION_LEAVE -> leaveFromInitialPosition(initialPositionChooser.getSelected());
                    case FALLBACK_LEAVE -> fallbackLeave();
                };

        boolean shouldScoreL1Coral = this.shouldScoreL1Coral.getBoolean(false);

        if (shouldScoreL1Coral) {
            auto = new ZeroElevatorCommand(elevator)
                    .andThen(Commands.deadline(
                            ElevatorCommands.waitForSetpoint(elevator, ElevatorSetpoint.L1_CORAL)
                                    .andThen(auto.asProxy())
                                    .andThen(Commands.run(
                                                    () -> endEffector.setEndEffector(-MoPrefs.autoExtakeCoralPower
                                                            .get()
                                                            .in(Units.Value)),
                                                    endEffector)
                                            .withTimeout(MoPrefs.autoExtakePreloadTime.get())),
                            ElevatorCommands.holdSetpoint(elevator, ElevatorSetpoint.L1_CORAL)));
        }

        return auto;
    }
}
