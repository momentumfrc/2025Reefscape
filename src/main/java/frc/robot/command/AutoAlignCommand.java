package frc.robot.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.SwerveSubsystem;
import frc.robot.vision.LimelightLocalization;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import java.util.List;
import java.util.Map;

public class AutoAlignCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final LimelightLocalization limelightLocalization;
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;
    private final Map<String, List<Translation2d>> reefPositionGroups;

    private Translation2d targetPosition;
    private final String targetGroupKey;
    private boolean fineAlignActive = false;

    public AutoAlignCommand(SwerveSubsystem swerveSubsystem, LimelightLocalization limelightLocalization,
                             Map<String, List<Translation2d>> reefPositionGroups, String targetGroupKey) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightLocalization = limelightLocalization;
        this.reefPositionGroups = reefPositionGroups;
        this.targetGroupKey = targetGroupKey;

        xController = new PIDController(1.0, 0.0, 0.0);
        yController = new PIDController(1.0, 0.0, 0.0);
        thetaController = new ProfiledPIDController(2.0, 0.0, 0.0,
                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(Math.PI, Math.PI / 2));

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d robotPose = limelightLocalization.getRobotPose(); // Get the robot's current position

        // Determine the target position based on the operator-selected group
        List<Translation2d> targetGroup = reefPositionGroups.get(targetGroupKey);
        if (targetGroup != null && !targetGroup.isEmpty()) {
            targetPosition = getClosestTarget(robotPose.getTranslation(), targetGroup);
        } else {
            throw new IllegalArgumentException("Target group positions must be set and valid.");
        }
    }

    @Override
    public void execute() {
        if (targetPosition == null) return;

        if (!fineAlignActive) {
            Pose2d currentPose = swerveSubsystem.getPose();
            Pose2d targetPose = new Pose2d(targetPosition, Rotation2d.fromDegrees(0)); // Align facing forward

            // Generate a trajectory to align with the target
            PathPlannerTrajectory trajectory = PathPlanner.generateTrajectory(
                    currentPose,
                    List.of(),
                    targetPose,
                    new TrajectoryConfig(3.0, 2.0)
            );

            PPSwerveControllerCommand controllerCommand = new PPSwerveControllerCommand(
                    trajectory,
                    swerveSubsystem::getPose,
                    swerveSubsystem.getKinematics(),
                    xController,
                    yController,
                    thetaController,
                    swerveSubsystem::setModuleStates,
                    swerveSubsystem
            );

            controllerCommand.schedule();

            fineAlignActive = true; // Trigger fine alignment after trajectory
        } else {
            // Fine alignment step using PID controllers
            Pose2d currentPose = swerveSubsystem.getPose();
            double xError = targetPosition.getX() - currentPose.getX();
            double yError = targetPosition.getY() - currentPose.getY();

            double xCorrection = xController.calculate(currentPose.getX(), targetPosition.getX());
            double yCorrection = yController.calculate(currentPose.getY(), targetPosition.getY());

            swerveSubsystem.drive(yCorrection, xCorrection, 0.0, true); // Apply corrections for fine alignment
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        fineAlignActive = false; // Reset fine alignment state
    }

    @Override
    public boolean isFinished() {
        return false; // Continues as long as the button is held
    }

    private Translation2d getClosestTarget(Translation2d robotPosition, List<Translation2d> targets) {
        Translation2d closestTarget = targets.get(0);
        double minDistance = robotPosition.getDistance(closestTarget);

        for (Translation2d target : targets) {
            double distance = robotPosition.getDistance(target);
            if (distance < minDistance) {
                minDistance = distance;
                closestTarget = target;
            }
        }

        return closestTarget;
    }
}
