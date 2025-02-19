package frc.robot.command;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/**
 * PPSwerveControllerCommand follows a given trajectory using a holonomic drive controller.
 * It samples the trajectory, calculates the desired chassis speeds, converts them to swerve
 * module states using the provided kinematics, and sends the commands via outputModuleStates.
 */
public class PPSwerveControllerCommand extends Command {
    private final Trajectory trajectory;
    private final Supplier<Pose2d> poseSupplier;
    private final SwerveDriveKinematics kinematics;
    private final PPHolonomicDriveController driveController;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final Timer timer = new Timer();

    /**
     * Constructs a new PPSwerveControllerCommand.
     *
     * @param trajectory         The trajectory to follow.
     * @param poseSupplier       A supplier for the current robot pose.
     * @param kinematics         The swerve drive kinematics.
     * @param driveController    The PPHolonomicDriveController used to compute chassis speeds.
     * @param outputModuleStates A consumer that accepts the desired swerve module states.
     * @param drive              The drive subsystem (used as a requirement).
     */
    public PPSwerveControllerCommand(
            Trajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            SwerveDriveKinematics kinematics,
            PPHolonomicDriveController driveController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            SubsystemBase drive) {
        this.trajectory = trajectory;
        this.poseSupplier = poseSupplier;
        this.kinematics = kinematics;
        this.driveController = driveController;
        this.outputModuleStates = outputModuleStates;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        Trajectory.State desiredState = trajectory.sample(currentTime);
        Pose2d currentPose = poseSupplier.get();

        // Calculate chassis speeds using the holonomic drive controller.
        ChassisSpeeds chassisSpeeds = driveController.calculate(
                currentPose,
                desiredState,
                desiredState.poseMeters.getRotation()
        );

        // Convert chassis speeds into individual swerve module states.
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        // Desaturate wheel speeds based on your maximum drive speed (adjust 3.0 as needed).
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 3.0);

        // Output the calculated module states.
        outputModuleStates.accept(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        // Stop the drivetrain by sending zero speeds to all modules.
        SwerveModuleState[] stopStates = new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d()),
            new SwerveModuleState(0, new Rotation2d()),
            new SwerveModuleState(0, new Rotation2d()),
            new SwerveModuleState(0, new Rotation2d())
        };
        outputModuleStates.accept(stopStates);
    }
}
