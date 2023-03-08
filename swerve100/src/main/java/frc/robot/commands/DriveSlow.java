// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ResourceBundle.Control;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import team100.control.DualXboxControl;

public class DriveSlow extends CommandBase {
  /** Creates a new DriveSlow. */

  SwerveDriveSubsystem m_robotDrive;
  DualXboxControl m_control;
  public DriveSlow(SwerveDriveSubsystem robotDrive, DualXboxControl control) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = robotDrive;
    m_control = control;

    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.drive(
            m_control.xLimited(),
            m_control.yLimited(),
            m_control.rotLimited(),
            true
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
