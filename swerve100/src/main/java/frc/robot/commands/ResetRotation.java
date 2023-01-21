// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve2DriveSubsystem;

public class ResetRotation extends CommandBase {
  /** Creates a new ResetRotation. */
  Swerve2DriveSubsystem m_robotDrive;
  boolean done = false;

  public ResetRotation(Swerve2DriveSubsystem swerve2DriveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = swerve2DriveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.resetAHRS2();
    done = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
