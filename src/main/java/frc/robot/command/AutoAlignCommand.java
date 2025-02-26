package frc.robot.command;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.utils.PathPlannerCommands;
import java.util.List;
import java.util.Map;

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

    // Selected auto-align mode from the driver's button press
    private AutoAlignMode selectedMode;

    // Trajectories for the two phases
    private PathPlannerPath path;

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
    private final Map<AutoAlignMode, Translation2d> subNodeOffsets = Map.of(
            AutoAlignMode.CORAL_LEFT, new Translation2d(-0.3, 0.0),
            AutoAlignMode.CORAL_RIGHT, new Translation2d(0.3, 0.0),
            AutoAlignMode.ALGAE_MIDDLE, new Translation2d(0.0, 0.3));

    public AutoAlignCommand(DriveSubsystem drive, PositioningSubsystem positioning, AutoAlignMode selectedMode) {
        this.drive = drive;
        this.positioning = positioning;
        this.selectedMode = selectedMode;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
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

        // Determine the desired sub node offset based on the selected mode.
        Translation2d subOffset = subNodeOffsets.get(selectedMode);
        // Compute final target for the second phase:
        Pose2d subTarget = new Pose2d(closestMaster.getTranslation().plus(subOffset), closestMaster.getRotation());

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(closestMaster, subTarget);

        PathConstraints constraints = new PathConstraints(
                MoPrefs.swerveMaxLinearSpeed.get().in(MetersPerSecond),
                3,
                MoPrefs.swerveMaxAngularSpeed.get().in(RadiansPerSecond),
                4 * Math.PI);

        path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, Constants.ZERO_ROTATION));

        PathPlannerCommands.getFollowPathCommand(drive, positioning, path, false);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends.
        drive.driveCartesian(0, 0, 0);
    }

    /**
     * Auto-align modes corresponding to the three buttons.
     */
    public enum AutoAlignMode {
        CORAL_LEFT,
        CORAL_RIGHT,
        ALGAE_MIDDLE
    }
}
