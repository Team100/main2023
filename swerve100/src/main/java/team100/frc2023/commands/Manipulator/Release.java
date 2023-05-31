package team100.frc2023.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team100.frc2023.subsystems.Manipulator;

public class Release extends CommandBase {
  /** Creates a new Release. */

  Manipulator m_manipulator;

  public Release(Manipulator manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_manipulator = manipulator;
    addRequirements(m_manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulator.pinch(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_manipulator.pinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
