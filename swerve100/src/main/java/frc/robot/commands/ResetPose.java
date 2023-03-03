// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ResetPose extends CommandBase {
  /** Creates a new ResetPose. */
  public double x = 0;
  public double y = 0;
  public double theta = 0;
  public SwerveDriveSubsystem m_robotDrive;
  public boolean done = false;
  public ResetPose(SwerveDriveSubsystem robotDrive, double x, double y, double theta) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.x = x;
    this.y = y;
    m_robotDrive = robotDrive;
    this.theta = theta;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Translation2d newTranslation2d = new Translation2d(x, y);
    Rotation2d newRotation = new Rotation2d(theta);

    m_robotDrive.resetPose(new Pose2d(newTranslation2d, newRotation));
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
