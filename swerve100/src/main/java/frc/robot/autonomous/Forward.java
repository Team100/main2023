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

/** Add your docs here. */
public class Forward extends TrajectoryCommand {

    public Forward(SwerveDriveSubsystem m_robotDrive, Trajectory trajectory) {
        super(m_robotDrive, trajectory);
        
    }
    static Forward newForward(SwerveDriveSubsystem m_robotDrive, double x) {
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
                    currentRobotPose,
                    List.of(),
                    new Pose2d(xRobot + x, yRobot, new Rotation2d(0)),
                    trajectoryConfig);
    
            return new Forward(m_robotDrive, exampleTrajectory);
       } 
}
