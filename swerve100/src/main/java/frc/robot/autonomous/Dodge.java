// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class Dodge extends TrajectoryCommand {

  public Dodge(SwerveDriveSubsystem m_robotDrive, Trajectory trajectory) {
    super(m_robotDrive, trajectory);
  }

  static Dodge newDodge(SwerveDriveSubsystem m_robotDrive, double y) {

      Pose2d currentRobotPose = m_robotDrive.getPose();
      double xRobot = currentRobotPose.getX();
      double yRobot = currentRobotPose.getY();


      
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
              4,
              3)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(SwerveDriveSubsystem.kDriveKinematics);

      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(xRobot, yRobot, new Rotation2d(currentRobotPose.getRotation().getRadians() + Math.PI/2)),
              List.of(),
              new Pose2d(xRobot, yRobot + y, new Rotation2d(currentRobotPose.getRotation().getRadians() + Math.PI/2)),
              trajectoryConfig);

        return new Dodge(m_robotDrive, exampleTrajectory);

    }
}
