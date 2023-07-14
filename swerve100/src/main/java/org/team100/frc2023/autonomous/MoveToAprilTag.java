package org.team100.frc2023.autonomous;

import java.util.List;
import java.util.function.Supplier;

import org.team100.frc2023.commands.SwerveControllerCommand;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.subsystems.DriveControllers;
import org.team100.lib.subsystems.RedundantGyro;
import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class MoveToAprilTag extends SwerveControllerCommand {
    private static final double speedMetersPerSecond = 2;
    private static final double accelerationMetersPerSecondSquared = 1;
    private static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            speedMetersPerSecond,
            accelerationMetersPerSecondSquared)
            .setKinematics(SwerveDriveSubsystem.kDriveKinematics);


    public MoveToAprilTag(
            SwerveDriveSubsystem m_robotDrive,
            DriveControllers controllers,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Supplier<Pose2d> getPose,
            int tagID,
            RedundantGyro gyro) {
        super(
                genTrajectory(m_robotDrive, layout, getPose, tagID),
                m_robotDrive::getPose,
                SwerveDriveSubsystem.kDriveKinematics,
                controllers.xController,
                controllers.yController,
                controllers.thetaController,
                () -> new Rotation2d(),
                m_robotDrive::setModuleStates,
                gyro,
                m_robotDrive);
        addRequirements(m_robotDrive);
    }

    private static Trajectory genTrajectory(
            SwerveDriveSubsystem m_robotDrive,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Supplier<Pose2d> getPose,
            int tagID) {
        Pose2d aprilPose = layout.getTagPose(tagID).get().toPose2d();

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                getPose.get(),
                List.of(),
                new Pose2d(
                        aprilPose.getX() - 1,
                        aprilPose.getY(),
                        new Rotation2d(aprilPose.getRotation().getDegrees())),
                trajectoryConfig);

        return exampleTrajectory;
    }
}
