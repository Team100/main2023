// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoLevel;
import frc.robot.subsystems.SwerveDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VasiliAutonomous extends SequentialCommandGroup {
  /** Creates a new autonomous. */
  public VasiliAutonomous(SwerveDriveSubsystem m_robotDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Forward.newForward(m_robotDrive, 3), 
      new Rotate(m_robotDrive, Math.PI)
    //   new autoLevel(m_robotDrive.m_gyro, m_robotDrive)
    );
    

   }
//    static Trajectory makeTrajectory(SwerveDriveSubsystem m_robotDrive, double x) {
//     Pose2d currentRobotPose = m_robotDrive.getPose();
//         double xRobot = currentRobotPose.getX();
//         double yRobot = currentRobotPose.getY();
        
//     TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//                 4,
//                 3)
//                 // Add kinematics to ensure max speed is actually obeyed
//                 .setKinematics(SwerveDriveSubsystem.kDriveKinematics);

//         Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//                 // Start at the origin facing the +X direction
//                 currentRobotPose,
//                 List.of(),
//                 new Pose2d(xRobot + x, yRobot, new Rotation2d(0)),
//                 trajectoryConfig);

//         return exampleTrajectory;
//    } 
 }
