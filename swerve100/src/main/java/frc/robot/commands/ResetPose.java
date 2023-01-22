// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve2DriveSubsystem;

public class ResetPose extends CommandBase {
  /** Creates a new ResetAngle. */
  Swerve2DriveSubsystem robotDrive;
  Pose2d robotPose;
  boolean done = false;
  public ResetPose(Swerve2DriveSubsystem swerve2DriveSubsystem, Pose2d pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    robotDrive = swerve2DriveSubsystem;
    robotPose = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotDrive.resetPose(robotPose);
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
