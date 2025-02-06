// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import java.util.Optional;

public class PathPlannerCommands {
    public static Command getFollowPathCommand(
            DriveSubsystem drive,
            PositioningSubsystem positioning,
            PathPlannerPath path,
            boolean shouldAssumeRobotIsAtStart) {
        // Manually flip the path ourselves, since we need the starting pose
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red && !path.preventFlipping) {
            path = path.flipPath();
        }

        Optional<Pose2d> startPose = path.getStartingHolonomicPose();

        double driveBaseRadius = DriveSubsystem.SWERVE_WHEEL_OFFSET.in(Units.Meters) * Math.sqrt(2);

        // TODO: figure this out
        PathFollowingController controller = drive.getDriveController();
        RobotConfig config = Constants.pathPlannerRobotConfig;

        FollowPathCommand driveControllerCommand = new FollowPathCommand(
                path,
                positioning::getRobotPose,
                drive::getRobotRelativeSpeeds,
                drive::driveRobotRelativeSpeeds,
                controller,
                config,
                () -> false, // Do not allow PathPlanner to flip the path, we already did manually
                drive);

        if (shouldAssumeRobotIsAtStart && startPose.isPresent()) {
            return new SequentialCommandGroup(
                    new InstantCommand(() -> positioning.setRobotPose(startPose.get())), driveControllerCommand);
        } else {
            return driveControllerCommand;
        }
    }

    public static Command getFollowPathCommand(
            DriveSubsystem drive,
            PositioningSubsystem positioning,
            String pathName,
            boolean shouldAssumeRobotIsAtStart) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
            return getFollowPathCommand(drive, positioning, path, shouldAssumeRobotIsAtStart);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load autonomous path: " + e.getLocalizedMessage(), e.getStackTrace());
            return Commands.print("Failed to load autonomous path!");
        }
    }
}
