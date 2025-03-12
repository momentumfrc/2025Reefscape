// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;

public class PathPlannerCommands {
    public static Command getFollowPathCommand(
            DriveSubsystem drive, PositioningSubsystem positioning, PathPlannerPath path) {
        PathFollowingController controller = drive.getDriveController();
        RobotConfig config = Constants.pathPlannerRobotConfig;

        FollowPathCommand driveControllerCommand = new FollowPathCommand(
                path,
                positioning::getRobotPose,
                drive::getRobotRelativeSpeeds,
                drive::driveRobotRelativeSpeeds,
                controller,
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drive);

        return driveControllerCommand;
    }

    public static Command getFollowPathCommand(
            DriveSubsystem drive, PositioningSubsystem positioning, String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
            return getFollowPathCommand(drive, positioning, path);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load autonomous path: " + e.getLocalizedMessage(), e.getStackTrace());
            return Commands.print("Failed to load autonomous path!");
        }
    }
}
