// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand2;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autonomous extends SequentialCommandGroup {
  /** Creates a new autonomous. */
  public autonomous(SwerveControllerCommand2 swerveControllerCommand, SwerveControllerCommand2 swerveControllerCommand2) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(swerveControllerCommand);
    addCommands(new WaitCommand(5));
    addCommands(swerveControllerCommand2);

  }
}
