package org.team100.frc2023.commands;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveMobility extends CommandBase {
  /** Creates a new DriveMobility. */
  boolean done = false;
  SwerveDriveSubsystem m_robotDrive;
  ProfiledPIDController m_headingController;
  public DriveMobility(SwerveDriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_headingController = driveSubsystem.headingController;
    m_headingController.enableContinuousInput(-Math.PI, Math.PI);

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

        // Pose2d currentPose = m_robotDrive.getPose();

        
        // double currentRads = MathUtil.angleModulus(currentPose.getRotation().getRadians());

        // double thetaControllerOutput = m_headingController.calculate(currentRads, 0);
        // m_robotDrive.resetPose(currentPose.getX(), currentPose.getY(), ;
        m_robotDrive.drive(0.3, 0, 0, true); //make false 


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
