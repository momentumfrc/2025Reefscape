package frc.robot.utils;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.DriveSubsystem;
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
        RED_WALL(new Pose2d(7.5, 0.5, Rotation2d.kZero));

        public final Pose2d pose;

        private InitialPosition(Pose2d pose) {
            this.pose = pose;
        }
    }

    private final GenericEntry masterAutoSwitch;
    private final SendableChooser<AutoChoices> autoChoicesChooser = MoShuffleboard.enumToChooser(AutoChoices.class);
    private final SendableChooser<InitialPosition> initialPositionChooser =
            MoShuffleboard.enumToChooser(InitialPosition.class);

    private final PositioningSubsystem positioning;
    private final DriveSubsystem drive;

    public AutoChooser(PositioningSubsystem positioning, DriveSubsystem drive) {
        this.positioning = positioning;
        this.drive = drive;

        masterAutoSwitch = MoShuffleboard.getInstance()
                .autoTab
                .add("Enable Auto?", true)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        MoShuffleboard.getInstance().autoTab.add("Auto Choices", autoChoicesChooser);
        MoShuffleboard.getInstance().autoTab.add("Initial Position", initialPositionChooser);
    }

    private Transform2d getLeaveTransform() {
        Transform2d transform = new Transform2d(MoPrefs.autoLeaveDist.get().in(Units.Meters), 0, Rotation2d.kZero);
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return transform;
        } else {
            return transform.inverse();
        }
    }

    private Command leaveFromPose(Pose2d initialPos, boolean assumeRobotAtPos) {
        Transform2d transform = getLeaveTransform();
        Pose2d destPos = initialPos.plus(transform);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(initialPos, destPos);
        PathConstraints constraints = new PathConstraints(
                MoPrefs.autoMaxLinVel.get(),
                MoPrefs.autoMaxLinAccel.get(),
                MoPrefs.autoMaxAngVel.get(),
                MoPrefs.autoMaxAngAccel.get());
        PathPlannerPath path =
                new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, initialPos.getRotation()));
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
        return leaveFromPose(pos.pose, true);
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

        return switch (autoChoicesChooser.getSelected()) {
            case DYNAMIC_LEAVE -> dynamicLeave();
            case INITIAL_POSITION_LEAVE -> leaveFromInitialPosition(initialPositionChooser.getSelected());
            case FALLBACK_LEAVE -> fallbackLeave();
        };
    }
}
