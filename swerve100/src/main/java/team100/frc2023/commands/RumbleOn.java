package team100.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team100.frc2023.control.DualXboxControl;

public class RumbleOn extends CommandBase {
  /** Creates a new RumbleOn. */

  DualXboxControl m_control;
  public RumbleOn(DualXboxControl control) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_control = control;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.out.println("*************************");
    m_control.rumbleOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_control.rumbleOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
