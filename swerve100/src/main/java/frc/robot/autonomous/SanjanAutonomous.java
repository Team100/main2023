// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoLevel;
import frc.robot.subsystems.SwerveDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SanjanAutonomous extends SequentialCommandGroup {
  /** Creates a new autonomous. */
  public SanjanAutonomous(SwerveDriveSubsystem m_robotDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //   new Forward(m_robotDrive, 0.5),
    //   new WaitCommand(5),
    //   new MoveToAprilTag(m_robotDrive, 3)
    //   new Forward(m_robotDrive, 4)
    // new Forward(m_robotDrive, 1),
     new AutoLevel(m_robotDrive.m_gyro, m_robotDrive)
    );
    

   }
 }
