// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.List;
import java.util.function.Supplier;

import javax.management.DescriptorRead;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class moveFromStartingPoseToGamePiece extends TrajectoryCommand {
    private double IsRunning = 5;
  /** Creates a new moveFromStartingPoseToGamePiece. */
  public moveFromStartingPoseToGamePiece(SwerveDriveSubsystem m_robotDrive, Trajectory trajectory, Supplier<Rotation2d> desiredRotation) {
    super(m_robotDrive, trajectory, desiredRotation);
    SmartDashboard.putData("Move From Starting Pose To Game Piece", this);
  }
  public static moveFromStartingPoseToGamePiece newMoveFromStartingPoseToGamePiece(SwerveDriveSubsystem m_robotDrive,
   Pose2d startingPose, Pose2d targetPose){
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        5,
    5)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(SwerveDriveSubsystem.kDriveKinematics);
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            startingPose,
            List.of(),
            targetPose,
            trajectoryConfig);
        System.out.println(exampleTrajectory);
    return new moveFromStartingPoseToGamePiece(m_robotDrive, exampleTrajectory, () -> new Rotation2d());
  }

  @Override
  public void execute() {
    super.execute();
    System.out.println("WREGERGERUERzZGRHERIUGRE");
    IsRunning = 5;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted); 
    IsRunning = 0;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("IsRunning", () -> IsRunning, null);
  }
}