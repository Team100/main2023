// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.autoLevel;
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
        new DriveToWaypoint2(new Pose2d(-1, 0, new Rotation2d()), m_robotDrive)
    //   Forward.newForward(m_robotDrive, -3), 
    //   new Rotate(m_robotDrive, Math.PI)
    //   new WaitCommand(1),
    //   new Rotate(m_robotDrive, Math.PI),
    //   Forward.newForward(m_robotDrive, 3),
    //   new autoLevel(m_robotDrive.m_gyro, m_robotDrive)
        // Dodge.newDodge(m_robotDrive, 3),
        // Dodge.newDodge(m_robotDrive, -3)
    );
   }
 }
