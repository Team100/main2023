package org.team100.frc2023.autonomous;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveStop extends CommandBase {
  /** Creates a new DriveStop. */

  SwerveDriveSubsystem m_robotDrive;
  public DriveStop(SwerveDriveSubsystem robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = robotDrive;

    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.drive(0, 0, 0, true);
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
