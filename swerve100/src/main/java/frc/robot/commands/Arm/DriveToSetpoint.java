// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.ArmController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToSetpoint extends ParallelCommandGroup {
  /** Creates a new DriveToSetpoint. */
  public DriveToSetpoint(ArmController arm, double x, double y, double upperVelocity, double lowerVelocity) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      UpperArmToGoal.factory(arm.calculate(x, y).upperTheta, arm, upperVelocity),
      LowerArmToGoal.factory(arm.calculate(x, y).lowerTheta, arm)

    );
  }
}
