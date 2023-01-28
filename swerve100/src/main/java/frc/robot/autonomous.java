// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand2;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve2DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autonomous extends SequentialCommandGroup {

  /** Creates a new autonomous. */
  public autonomous(Swerve2DriveSubsystem m_robotDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(generateSwerveControllerCommand1(m_robotDrive));
    addCommands(new WaitCommand(5));
    addCommands(generateSwerveControllerCommand2(m_robotDrive));

  }

  private SwerveControllerCommand2 generateSwerveControllerCommand1(Swerve2DriveSubsystem m_robotDrive ){
    Pose2d newRobotPose = new Pose2d(m_robotDrive.getPose().getX() + 10, m_robotDrive.getPose().getY(), m_robotDrive.getPose().getRotation() );
    TrajectoryConfig config =
          new TrajectoryConfig(
                  AutoConstants.kMaxSpeedMetersPerSecond,
                  AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(Swerve2DriveSubsystem.kDriveKinematics);
    Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direc  tion
          m_robotDrive.getPose(),
          // m_robotDrive.getPose(),
          // new Pose2d(0 ,0 , new Rotation2d(0)),
          List.of(),
          // new Pose2d(aprilPose.getX() - 1, aprilPose.getY() , new Rotation2d(0)),
          // new Pose2d(10, 0, new Rotation2d(0)),
          // Pass config
          newRobotPose,
          config);

    SwerveControllerCommand2 swerveControllerCommand =
        new SwerveControllerCommand2(
          exampleTrajectory,
          m_robotDrive::getPose, // Functional interface to feed supplier
          Swerve2DriveSubsystem.kDriveKinematics,

          // Position controllers
          m_robotDrive.xController,
          m_robotDrive.yController,
          m_robotDrive.thetaController,
          () -> new Rotation2d(),
          m_robotDrive::setModuleStates,
          m_robotDrive);

    return swerveControllerCommand;
  }

  private SwerveControllerCommand2 generateSwerveControllerCommand2(Swerve2DriveSubsystem m_robotDrive ){
    
    Pose2d aprilPose = new Pose2d(0, 0, new Rotation2d(0));

    TrajectoryConfig config =
          new TrajectoryConfig(
                  AutoConstants.kMaxSpeedMetersPerSecond,
                  AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(Swerve2DriveSubsystem.kDriveKinematics);

    Trajectory exampleTrajectory2 =
      TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direc  tion
          m_robotDrive.getPose(),
          // new Pose2d(0, 0, new Rotation2d(0)),
          List.of(),
          new Pose2d(aprilPose.getX() - 1, aprilPose.getY() , new Rotation2d(0)),
          // new Pose2d(8, 0, new Rotation2d(0)),
          // Pass config
          config);

    SwerveControllerCommand2 swerveControllerCommand =
        new SwerveControllerCommand2(
          exampleTrajectory2,
          m_robotDrive::getPose, // Functional interface to feed supplier
          Swerve2DriveSubsystem.kDriveKinematics,

          // Position controllers
          m_robotDrive.xController,
          m_robotDrive.yController,
          m_robotDrive.thetaController,
          () -> new Rotation2d(),
          m_robotDrive::setModuleStates,
          m_robotDrive);

    return swerveControllerCommand;
  }

  
}
