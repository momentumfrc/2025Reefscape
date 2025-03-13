package frc.robot.utils;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

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
import frc.robot.command.elevator.ElevatorCommands;
import frc.robot.command.elevator.ZeroElevatorCommand;
import frc.robot.component.ElevatorSetpointManager.ElevatorSetpoint;
import frc.robot.component.FieldGeometry;
import frc.robot.component.FieldGeometry.ReefFace;
import frc.robot.molib.MoShuffleboard;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.EndEffectorSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.List;

public class AutoChooser {
    private enum AutoChoices {
        FALLBACK_LEAVE,
        ALIGN_REEF,
        LEAVE
    }

    private enum InitialPosition {
        BLUE_WALL,
        CENTER,
        RED_WALL;
    }

    private enum DriverRelativeInitialPosition {
        RIGHT_WALL,
        CENTER,
        LEFT_WALL;
    }

    private final GenericEntry masterAutoSwitch;
    private final SendableChooser<AutoChoices> autoChoicesChooser = MoShuffleboard.enumToChooser(AutoChoices.class);
    private final SendableChooser<InitialPosition> initialPositionChooser =
            MoShuffleboard.enumToChooser(InitialPosition.class);
    private final SendableChooser<FieldGeometry.ReefFace> destChooser =
            MoShuffleboard.enumToChooser(FieldGeometry.ReefFace.class);
    private final GenericEntry shouldScoreL1Coral;
    private final GenericEntry shouldAssumeInitialPos;

    private final EnumMap<DriverRelativeInitialPosition, EnumSet<FieldGeometry.ReefFace>> permissibleTargets =
            new EnumMap<>(DriverRelativeInitialPosition.class);

    private final GenericEntry scoreL2CoralCycle;

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
        MoShuffleboard.getInstance().autoTab.add("Destination", destChooser);

        shouldScoreL1Coral = MoShuffleboard.getInstance()
                .autoTab
                .add("Score L1 preload?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();
        shouldAssumeInitialPos = MoShuffleboard.getInstance()
                .autoTab
                .add("Assume robot at initial position?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        permissibleTargets.put(
                DriverRelativeInitialPosition.LEFT_WALL, EnumSet.of(ReefFace.KL, ReefFace.IJ, ReefFace.GH));
        permissibleTargets.put(DriverRelativeInitialPosition.CENTER, EnumSet.of(ReefFace.IJ, ReefFace.GH, ReefFace.EF));
        permissibleTargets.put(
                DriverRelativeInitialPosition.RIGHT_WALL, EnumSet.of(ReefFace.GH, ReefFace.EF, ReefFace.CD));

        scoreL2CoralCycle = MoShuffleboard.getInstance()
                .autoTab
                .add("Cycle L2?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    positioning::getRobotPose,
                    positioning::setRobotPose,
                    drive::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> drive.driveRobotRelativeSpeeds(speeds),
                    new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    drive);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private Pose2d getPoseForInitialPosition(DriverRelativeInitialPosition position) {
        double halfWidth = MoPrefs.robotWidthWithBumpers.get().in(Units.Meters) / 2;
        return switch (position) {
            case LEFT_WALL -> new Pose2d(7.582, 8.056 - halfWidth, Rotation2d.kZero);
            case CENTER -> new Pose2d(7.582, 4.028, Rotation2d.kZero);
            case RIGHT_WALL -> new Pose2d(7.582, 0 + halfWidth, Rotation2d.kZero);
        };
    }

    private DriverRelativeInitialPosition relativeInitialPosForAbsolutePos(InitialPosition pos) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return switch (pos) {
                case BLUE_WALL -> DriverRelativeInitialPosition.LEFT_WALL;
                case CENTER -> DriverRelativeInitialPosition.CENTER;
                case RED_WALL -> DriverRelativeInitialPosition.RIGHT_WALL;
            };
        } else {
            return switch (pos) {
                case BLUE_WALL -> DriverRelativeInitialPosition.RIGHT_WALL;
                case CENTER -> DriverRelativeInitialPosition.CENTER;
                case RED_WALL -> DriverRelativeInitialPosition.LEFT_WALL;
            };
        }
    }

    /**
     * Gets the starting pose of the robot in ALLIANCE-RELATIVE COORDINATES.
     */
    private Pose2d getRobotPose(boolean assumeRobotAtPos) {
        if (assumeRobotAtPos || !positioning.hasInitialPosition()) {
            return getPoseForInitialPosition(relativeInitialPosForAbsolutePos(initialPositionChooser.getSelected()));
        } else {
            Pose2d pose = positioning.getRobotPose();
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                pose = FlippingUtil.flipFieldPose(pose);
            }
            return pose;
        }
    }

    /**
     * Accepts the pose of the robot in ALLIANCE-RELATIVE coordinates and updates the positioning subsystem
     * with the correct FIELD-RELATIVE coordinates.
     */
    private void setRobotPose(Pose2d pose) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            pose = FlippingUtil.flipFieldPose(pose);
        }
        positioning.setRobotPose(pose);
    }

    private Command planPath(IdealStartingState startingState, GoalEndState endState, Pose2d... poses) {
        if (poses.length < 2) {
            throw new IllegalArgumentException("Must have at least 2 poses to draw a path");
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);
        PathConstraints constraints = new PathConstraints(
                MoPrefs.autoMaxLinVel.get(),
                MoPrefs.autoMaxLinAccel.get(),
                MoPrefs.autoMaxAngVel.get(),
                MoPrefs.autoMaxAngAccel.get());
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, startingState, endState);

        return PathPlannerCommands.getFollowPathCommand(drive, positioning, path);
    }

    private Command alignReef() {
        boolean assumeRobotAtPos = shouldAssumeInitialPos.getBoolean(false);
        Pose2d robotPose = getRobotPose(assumeRobotAtPos);

        if (assumeRobotAtPos) {
            setRobotPose(robotPose);
        } else if (!positioning.hasInitialPosition()) {
            return Commands.print("No initial position established, using fallback autonomous.")
                    .andThen(fallbackLeave());
        }

        double minDistToInitialPosition = Double.MAX_VALUE;
        DriverRelativeInitialPosition currPosition = null;
        for (DriverRelativeInitialPosition position : DriverRelativeInitialPosition.values()) {
            Pose2d testPos = getPoseForInitialPosition(position);
            double dist = testPos.minus(robotPose).getTranslation().getNorm();
            if (dist < minDistToInitialPosition) {
                minDistToInitialPosition = dist;
                currPosition = position;
            }
        }

        Pose2d startPose = robotPose.plus(new Transform2d(0, 0, Rotation2d.k180deg));
        EnumSet<FieldGeometry.ReefFace> permissibleFaces = permissibleTargets.get(currPosition);
        FieldGeometry.ReefFace requestedFace = destChooser.getSelected();
        if (permissibleFaces.contains(requestedFace)) {
            // Always get blue alliance because all path planning is done in blue-alliance-relative coordinates, and
            // then, if necessary, the path is flipped to the red side once we start following
            Pose2d facePose = FieldGeometry.getInstance().getFacePose(requestedFace, Alliance.Blue);
            facePose = facePose.plus(
                    new Transform2d(MoPrefs.robotWidthWithBumpers.get().in(Units.Meters) / 2, 0, Rotation2d.k180deg));
            Pose2d approachPose = facePose.plus(
                    new Transform2d(-1 * MoPrefs.autoReefApproachDistance.get().in(Units.Meters), 0, Rotation2d.kZero));
            return planPath(
                    new IdealStartingState(0, robotPose.getRotation()),
                    new GoalEndState(0, facePose.getRotation().plus(Rotation2d.k180deg)),
                    startPose,
                    approachPose,
                    facePose);
        } else {
            DriverStation.reportWarning(
                    "Selected destination " + requestedFace.toString()
                            + " is not navigatable from current starting position " + currPosition
                            + ". Falling back to LEAVE autonomous.",
                    false);
            Transform2d transform = new Transform2d(MoPrefs.autoLeaveDist.get().in(Units.Meters), 0, Rotation2d.kZero);
            Pose2d destPose = startPose.plus(transform);
            return planPath(
                    new IdealStartingState(0, robotPose.getRotation()),
                    new GoalEndState(0, robotPose.getRotation()),
                    startPose,
                    destPose);
        }
    }

    private Command leave() {
        boolean assumeRobotAtPos = shouldAssumeInitialPos.getBoolean(false);
        Pose2d robotPose = getRobotPose(assumeRobotAtPos);

        if (assumeRobotAtPos) {
            setRobotPose(robotPose);
        } else if (!positioning.hasInitialPosition()) {
            return Commands.print("No initial position established, using fallback autonomous.")
                    .andThen(fallbackLeave());
        }

        Pose2d startPose = robotPose.plus(new Transform2d(0, 0, Rotation2d.k180deg));
        Transform2d transform = new Transform2d(MoPrefs.autoLeaveDist.get().in(Units.Meters), 0, Rotation2d.kZero);
        Pose2d destPose = startPose.plus(transform);

        return planPath(
                new IdealStartingState(0, robotPose.getRotation()),
                new GoalEndState(0, robotPose.getRotation()),
                startPose,
                destPose);
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
                    case ALIGN_REEF -> alignReef();
                    case LEAVE -> leave();
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

        boolean scoreL2CoralCycle = this.scoreL2CoralCycle.getBoolean(false);


        if (scoreL2CoralCycle) {
            auto = new ZeroElevatorCommand(elevator)
                    .andThen(PathPlannerCommands.getFollowPathCommand(drive, positioning, "TEST"));
        }

        return auto;
    }
}
