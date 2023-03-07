// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.List;
import java.util.function.Supplier;

import javax.management.DescriptorRead;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class moveFromStartingPoseToGamePiece extends TrajectoryCommand {
  /** Creates a new moveFromStartingPoseToGamePiece. */
  public moveFromStartingPoseToGamePiece(SwerveDriveSubsystem m_robotDrive, Trajectory trajectory, Supplier<Rotation2d> desiredRotation) {
    super(m_robotDrive, trajectory, desiredRotation);
  }
  public static moveFromStartingPoseToGamePiece newMoveFromStartingPoseToGamePiece(SwerveDriveSubsystem m_robotDrive,
   Supplier<Pose2d> getPose, Pose2d targetPose){
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        5,
        10)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(SwerveDriveSubsystem.kDriveKinematics);
        Pose2d robot = getPose.get();
        robot = new Pose2d(robot.getX(), robot.getY(), new Rotation2d());
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            robot,
            List.of(),
            targetPose,
            trajectoryConfig);
        System.out.println(exampleTrajectory);
    return new moveFromStartingPoseToGamePiece(m_robotDrive, exampleTrajectory, () -> new Rotation2d());
  }
}