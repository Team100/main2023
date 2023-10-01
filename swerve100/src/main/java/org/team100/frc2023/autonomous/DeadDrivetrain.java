// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2023.autonomous;

import edu.wpi.first.wpilibj.Timer;

import org.team100.frc2023.commands.DriveScaled;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DeadDrivetrain extends CommandBase {
  /** Creates a new DeadDrivetrain. */

  SwerveDriveSubsystem m_robotDrive;
  Timer m_timer;
  boolean done = false;
  public DeadDrivetrain(SwerveDriveSubsystem robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = robotDrive;
    m_timer = new Timer();
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_robotDrive.removeDefaultCommand();
    m_robotDrive.setDefaultCommand(new DriveScaled(null, m_robotDrive, null));
    done = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    // System.out.println(done);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
