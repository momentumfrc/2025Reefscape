package frc.robot.command;

import java.util.List;
import java.util.Map;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.input.ControllerInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.utils.Vec2;
import com.pathplanner.lib.PathPlanner;

/*
 Driver holds down one of three buttons
 The robot finds the closest master node
 The robot generates a rotational and translational absolute trajectory to closest master node
 The robot generates a translation relavative trajectory to the sub node based button held by the driver
 The command ends when the final position is reached.
 */
public class AutoAlignCommand extends Command {
  private final DriveSubsystem drive;
  private final PositioningSubsystem positioning;
  private final ControllerInput controller;

  // Selected auto-align mode from the driver's button press
  private AutoAlignMode selectedMode;

  // Trajectories for the two phases
  private Trajectory absoluteTrajectory;
  private Trajectory relativeTrajectory;

  // A sequential command that will follow both trajectories
  private Command trajectoryCommand;

  // Example master node positions (12 master nodes for 6 reef sides, 2 per side).
  // These should normally be read from an external configuration (e.g. Elastic Dashboard)
  // Here we provide dummy sample values (in meters) in field coordinates.
  private final Pose2d[] masterNodes = new Pose2d[] {
      new Pose2d(2.0, 7.0, Constants.ZERO_ROTATION),
      new Pose2d(4.0, 7.0, Constants.ZERO_ROTATION),
      new Pose2d(6.0, 7.0, Constants.ZERO_ROTATION),
      new Pose2d(2.0, 5.0, Constants.ZERO_ROTATION),
      new Pose2d(4.0, 5.0, Constants.ZERO_ROTATION),
      new Pose2d(6.0, 5.0, Constants.ZERO_ROTATION),
      new Pose2d(2.0, 3.0, Constants.ZERO_ROTATION),
      new Pose2d(4.0, 3.0, Constants.ZERO_ROTATION),
      new Pose2d(6.0, 3.0, Constants.ZERO_ROTATION),
      new Pose2d(2.0, 1.0, Constants.ZERO_ROTATION),
      new Pose2d(4.0, 1.0, Constants.ZERO_ROTATION),
      new Pose2d(6.0, 1.0, Constants.ZERO_ROTATION)
  };

  // Mapping from auto-align mode to a relative offset for the final sub node.
  // These offsets are translational adjustments (in meters) relative to the master node.
  private final Map<AutoAlignMode, Translation2d> subNodeOffsets = new HashMap<>() {{
      put(AutoAlignMode.CORAL_LEFT, new Translation2d(-0.3, 0.0));
      put(AutoAlignMode.CORAL_RIGHT, new Translation2d(0.3, 0.0));
      put(AutoAlignMode.ALGAE_MIDDLE, new Translation2d(0.0, 0.3));
  }};

  // Trajectory configuration taken from settings.json values (adjust as necessary)
  private final TrajectoryConfig trajConfig = new TrajectoryConfig(3.0, 3.0)
          .setKinematics(drive.kinematics);

  public AutoAlignCommand(DriveSubsystem drive, PositioningSubsystem positioning, ControllerInput controller) {
    this.drive = drive;
    this.positioning = positioning;
    this.controller = controller;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Determine which auto-align button is pressed; assume only one is held.
    if (controller.getAutoAlignCoralLeft()) {
      selectedMode = AutoAlignMode.CORAL_LEFT;
    } else if (controller.getAutoAlignCoralRight()) {
      selectedMode = AutoAlignMode.CORAL_RIGHT;
    } else if (controller.getAutoAlignAlgaeMiddle()) {
      selectedMode = AutoAlignMode.ALGAE_MIDDLE;
    } else {
      // No button is held; cancel command.
      cancel();
      return;
    }

    // Get current robot pose from PositioningSubsystem
    Pose2d currentPose = positioning.getRobotPose();

    // Find the closest master node using Euclidean distance
    Pose2d closestMaster = null;
    double minDistance = Double.POSITIVE_INFINITY;
    for (Pose2d master : masterNodes) {
      double distance = currentPose.getTranslation().getDistance(master.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestMaster = master;
      }
    }

    if (closestMaster == null) {
      // Safety: if no master node is found, cancel command.
      cancel();
      return;
    }

    // Generate the absolute trajectory from the current pose to the closest master node.
    absoluteTrajectory = PathPlanner.generateTrajectory(
            currentPose,
            List.of(), // no intermediate waypoints
            closestMaster,
            trajConfig
    );

    // Determine the desired sub node offset based on the selected mode.
    Translation2d subOffset = subNodeOffsets.get(selectedMode);
    // Compute final target for the second phase:
    Pose2d subTarget = new Pose2d(closestMaster.getTranslation().plus(subOffset), closestMaster.getRotation());

    // Generate a relative trajectory from the master node to the sub node.
    // This trajectory is only translational; the rotation remains as the master node's rotation.
    relativeTrajectory = PathPlanner.generateTrajectory(
            closestMaster,
            List.of(), // no intermediate waypoints
            subTarget,
            trajConfig
    );

    // Create trajectory following commands.
    // Here we use PPSwerveControllerCommand (assumed to be configured similarly to your teleop drive controller)
    Command absoluteFollow = new PPSwerveControllerCommand(
            absoluteTrajectory,
            positioning::getPose,
            drive.kinematics,
            drive.getDriveController(),
            drive::driveSwerveStates,
            drive
    );

    Command relativeFollow = new PPSwerveControllerCommand(
            relativeTrajectory,
            // For the relative trajectory, we use a lambda that returns the master node pose
            // (the trajectory is relative; no additional rotation correction is needed)
            () -> closestMaster,
            drive.kinematics,
            drive.getDriveController(),
            drive::driveSwerveStates,
            drive
    );

    // Wrap each trajectory-following command with a condition that checks if the driver has released the auto-align button.
    Command absoluteFollowInterruptible = new ParallelRaceGroup(
            absoluteFollow,
            new WaitUntilCommand(() -> !isAutoAlignButtonHeld())
                    .withTimeout(absoluteTrajectory.getTotalTimeSeconds() + 0.1)
    );

    Command relativeFollowInterruptible = new ParallelRaceGroup(
            relativeFollow,
            new WaitUntilCommand(() -> !isAutoAlignButtonHeld())
                    .withTimeout(relativeTrajectory.getTotalTimeSeconds() + 0.1)
    );

    // Chain the two trajectory commands sequentially.
    trajectoryCommand = new SequentialCommandGroup(absoluteFollowInterruptible, relativeFollowInterruptible);

    // Schedule the internal trajectory command.
    trajectoryCommand.schedule();
  }

  @Override
  public void execute() {
    // Continually check for button release. If the driver releases the auto-align button, cancel the trajectory command.
    if (!isAutoAlignButtonHeld() && trajectoryCommand.isScheduled()) {
      trajectoryCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    // The AutoAlignCommand finishes when the entire trajectory command has completed.
    return trajectoryCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain when the command ends.
    drive.driveCartesian(0, 0, 0);
  }

  /**
   * Returns true if any of the auto-align buttons is held.
   */
  private boolean isAutoAlignButtonHeld() {
    return controller.getAutoAlignCoralLeft() ||
           controller.getAutoAlignCoralRight() ||
           controller.getAutoAlignAlgaeMiddle();
  }

  /**
   * Auto-align modes corresponding to the three buttons.
   */
  private enum AutoAlignMode {
    CORAL_LEFT,
    CORAL_RIGHT,
    ALGAE_MIDDLE
  }
}
