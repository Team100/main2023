// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2023.autonomous;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Empty extends CommandBase {
  /** Creates a new Empty. */
  SwerveDriveSubsystem m_robotDrive;
  public Empty(SwerveDriveSubsystem robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = robotDrive;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
