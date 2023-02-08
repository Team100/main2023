// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** Add your docs here. */
public class IshanAutonomous extends SwerveControllerCommand {

    public IshanAutonomous(SwerveDriveSubsystem m_robotDrive) {
        super(genTrajectory(m_robotDrive),
                m_robotDrive::getPose,
                SwerveDriveSubsystem.kDriveKinematics,
                m_robotDrive.xController,
                m_robotDrive.yController,
                m_robotDrive.thetaController,
                () -> new Rotation2d(),
                m_robotDrive::setModuleStates,
                m_robotDrive);
    }

    private static Trajectory genTrajectory(SwerveDriveSubsystem m_robotDrive) {
        double controlPointAngle = Math.atan2(
                (1.071626 - m_robotDrive.getPose().getY()),
                (14.513558 - m_robotDrive.getPose().getX())
        );
         final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(SwerveDriveSubsystem.kDriveKinematics);
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(m_robotDrive.getPose().getTranslation(), 
                new Rotation2d(controlPointAngle)),
                List.of(
                    new Translation2d(
                        (14.513558 + m_robotDrive.getPose().getX()) / 2,
                        (1.071626 + m_robotDrive.getPose().getY()) / 2)),
                    new Pose2d(14.513558, 1.071626, new Rotation2d(controlPointAngle)),
                kTrajectoryConfig
        );

        return exampleTrajectory;
    }
}
