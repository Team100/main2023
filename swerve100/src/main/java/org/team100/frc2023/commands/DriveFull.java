package org.team100.frc2023.commands;

import org.team100.frc2023.control.DualXboxControl;
import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveFull extends CommandBase {
  /** Creates a new DriveFull. */

  SwerveDriveSubsystem m_robotDrive;
  DualXboxControl m_control;
  public DriveFull(SwerveDriveSubsystem driveSubsystem, DualXboxControl control) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = driveSubsystem;

    addRequirements(driveSubsystem);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    double xVal = MathUtil.applyDeadband(m_control.xSpeed()/2, .02);

    double yVal = MathUtil.applyDeadband(m_control.xSpeed()/2, .02);

    xVal = Math.signum(xVal) * 1;

    yVal = Math.signum(yVal) * 1;

    m_robotDrive.drive(
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