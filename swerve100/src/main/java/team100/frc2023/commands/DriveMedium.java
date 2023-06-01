package team100.frc2023.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team100.frc2023.control.DualXboxControl;
import team100.frc2023.subsystems.SwerveDriveSubsystem;

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
    // m_robotDrive.driveMedium(
    //             MathUtil.applyDeadband(m_control.xSpeed()/2, .02),
    //             MathUtil.applyDeadband(m_control.ySpeed()/2, .02),
    //             MathUtil.applyDeadband(m_control.rotSpeed(), .02),
    //             true);

    double xVal = MathUtil.applyDeadband(m_control.xSpeed()/2, .02);

    double yVal = MathUtil.applyDeadband(m_control.ySpeed()/2, .02);

    xVal = Math.signum(xVal) * 1;

    yVal = Math.signum(yVal) * 1;

    m_robotDrive.drive(
                MathUtil.applyDeadband(xVal, .02),
                MathUtil.applyDeadband(yVal, .02),
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
