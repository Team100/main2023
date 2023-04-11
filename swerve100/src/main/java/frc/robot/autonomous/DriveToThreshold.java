// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveToThreshold extends CommandBase {
  /** Creates a new DriveToThreshold. */
  SwerveDriveSubsystem m_robotDrive;
  boolean done;
  public DriveToThreshold(SwerveDriveSubsystem robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = robotDrive;

    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_robotDrive.getPose().getX() > 4.1){
        m_robotDrive.drive(-0.4, 0, 0, true);
    } else {
      m_robotDrive.drive(0, 0, 0, true);
      done = true;
    }
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
