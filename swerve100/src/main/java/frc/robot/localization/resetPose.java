// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve2DriveSubsystem;

public class resetPose extends CommandBase {
  /** Creates a new resetPose. */
  boolean done = false;
  Swerve2DriveSubsystem driveSubsystem;
  public resetPose(Swerve2DriveSubsystem dS) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSubsystem = dS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetAHRS2();
    driveSubsystem.resetPose();
    done = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
