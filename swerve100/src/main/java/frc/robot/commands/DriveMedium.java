// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import team100.control.DualXboxControl;

public class DriveMedium extends CommandBase {
  /** Creates a new DriveMedium. */

  private final SwerveDriveSubsystem m_robotDrive;
  private final DualXboxControl m_control;

  public DriveMedium(SwerveDriveSubsystem robotDrive, DualXboxControl control) {
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
    m_robotDrive.driveMedium(
                MathUtil.applyDeadband(m_control.xSpeed()/2, .02),
                MathUtil.applyDeadband(m_control.ySpeed()/2, .02),
                MathUtil.applyDeadband(m_control.rotSpeed(), .02),
                true);
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
