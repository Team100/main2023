package org.team100.frc2023.autonomous;

import java.util.List;

import org.team100.frc2023.commands.SwerveControllerCommand;
import org.team100.frc2023.subsystems.SwerveDriveSubsystem;
import org.team100.lib.subsystems.RedundantGyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class IshanAutonomous extends SwerveControllerCommand {
    private static final double speedMetersPerSecond = 1;
    private static final double accelerationMetersPerSecondSquared = 1;

    private static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
            speedMetersPerSecond,
            accelerationMetersPerSecondSquared)
            .setKinematics(SwerveDriveSubsystem.kDriveKinematics);

    public IshanAutonomous(SwerveDriveSubsystem m_robotDrive, RedundantGyro gyro) {
        super(genTrajectory(m_robotDrive),
                m_robotDrive::getPose,
                SwerveDriveSubsystem.kDriveKinematics,
                m_robotDrive.controllers.xController,
                m_robotDrive.controllers.yController,
                m_robotDrive.controllers.thetaController,
                () -> new Rotation2d(),
                m_robotDrive::setModuleStates,
                gyro,
                m_robotDrive);
    }

    private static Trajectory genTrajectory(SwerveDriveSubsystem m_robotDrive) {
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
