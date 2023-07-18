package org.team100.frc2023.autonomous;

import java.util.List;
import java.util.function.Supplier;

import org.team100.frc2023.commands.SwerveControllerCommand;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class MoveToAprilTag extends SwerveControllerCommand {
    public static class Config {
        public double speedMetersPerSecond = 2;
        public double accelerationMetersPerSecondSquared = 1;
    }

    public MoveToAprilTag(
            Config config,
            SwerveDriveSubsystem m_robotDrive,
            SwerveDriveKinematics kinematics,
            DriveControllers controllers,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Supplier<Pose2d> getPose,
            int tagID,
            RedundantGyro gyro) {
        super(
                genTrajectory(config, m_robotDrive, kinematics, layout, getPose, tagID),
                m_robotDrive::getPose,
                kinematics,
                controllers,
                () -> new Rotation2d(),
                m_robotDrive::setModuleStates,
                gyro,
                m_robotDrive);
        addRequirements(m_robotDrive);
    }

    private static Trajectory genTrajectory(
            Config config,
            SwerveDriveSubsystem m_robotDrive,
            SwerveDriveKinematics kinematics,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Supplier<Pose2d> getPose,
            int tagID) {

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                config.speedMetersPerSecond,
                config.accelerationMetersPerSecondSquared)
                .setKinematics(kinematics);

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
