// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveMobility extends CommandBase {
  /** Creates a new DriveMobility. */
  boolean done = false;
  SwerveDriveSubsystem m_robotDrive;
  public DriveMobility(SwerveDriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = driveSubsystem;
    addRequirements(m_robotDrive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_robotDrive.getPose().getX() < 5.8){
        m_robotDrive.drive(0.3, 0, 0, true);
    } else {
      m_robotDrive.drive(0, 0, 0, true);
      done = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    m_robotDrive.drive(0, 0, 0, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
