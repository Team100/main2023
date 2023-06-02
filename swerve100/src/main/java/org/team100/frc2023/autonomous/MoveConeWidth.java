package org.team100.frc2023.autonomous;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveConeWidth extends CommandBase {
  /** Creates a new MoveConeWidth. */

  SwerveDriveSubsystem m_robotDrive;

  double goalX = 0;
  double goalY = 0;

  double goalRotation = Math.PI;

  double m_coneWidth;
  ProfiledPIDController yController;
  private final PIDController m_rotationController;

  public MoveConeWidth( SwerveDriveSubsystem robotDrive, double coneWidth) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = robotDrive;

    m_coneWidth = coneWidth * 0.155;

    yController = new ProfiledPIDController(2, 0, 0,  new Constraints(5,  0.7)  ); //0.7

    yController.setTolerance(0.00000001);

    m_rotationController = new PIDController(2, 0, 0); //4.5
    m_rotationController.setTolerance(Math.PI / 180);

    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    yController.reset(m_robotDrive.getPose().getY());
    goalX = m_robotDrive.getPose().getX() ;
    goalY = m_robotDrive.getPose().getY() + m_coneWidth ;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double outputRot = m_rotationController.calculate(m_robotDrive.getPose().getRotation().getRadians(), goalRotation);
    double outputY = yController.calculate(m_robotDrive.getPose().getY(), goalY);

    m_robotDrive.drive(0, outputY, outputRot, true);

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
