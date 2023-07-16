package org.team100.frc2023.autonomous;

import java.util.List;

import org.team100.frc2023.commands.SwerveControllerCommand;
import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class IshanAutonomous extends SwerveControllerCommand {
    public static class Config {
        public double speedMetersPerSecond = 1;
        public double accelerationMetersPerSecondSquared = 1;
    }

    public IshanAutonomous(
            Config m_config,
            SwerveDriveSubsystem m_robotDrive,
            SwerveDriveKinematics kinematics,
            DriveControllers controllers,
            RedundantGyro gyro) {
        super(genTrajectory(m_config, m_robotDrive, kinematics),
                m_robotDrive::getPose,
                kinematics,
                controllers.xController,
                controllers.yController,
                controllers.thetaController,
                () -> new Rotation2d(),
                m_robotDrive::setModuleStates,
                gyro,
                m_robotDrive);
    }

    private static Trajectory genTrajectory(Config config, SwerveDriveSubsystem m_robotDrive,
            SwerveDriveKinematics kinematics) {
        TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
                config.speedMetersPerSecond,
                config.accelerationMetersPerSecondSquared)
                .setKinematics(kinematics);
        double controlPointAngle = Math.atan2(
                (1.071626 - m_robotDrive.getPose().getY()),
                (14.513558 - m_robotDrive.getPose().getX()));

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(m_robotDrive.getPose().getTranslation(), new Rotation2d(controlPointAngle)),
                List.of(
                        new Translation2d(
                                (14.513558 + m_robotDrive.getPose().getX()) / 2,
                                (1.071626 + m_robotDrive.getPose().getY()) / 2)),
                new Pose2d(14.513558, 1.071626, new Rotation2d(controlPointAngle)),
                kTrajectoryConfig);

        return exampleTrajectory;
    }
}
