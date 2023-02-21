// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

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
        // new DriveToAprilTag(m_robotDrive, 5)
        Forward.newForward(m_robotDrive, -3),
        new Rotate(m_robotDrive, Math.PI),
        new Rotate(m_robotDrive, 0),
        Forward.newForward(m_robotDrive, 3)
    );  
   }
 }
