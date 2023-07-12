package org.team100.frc2023.commands.Manipulator;

import org.team100.frc2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Open extends CommandBase {
  /** Creates a new Open. */

  Manipulator m_manipulator;
  double openPosition;

  public Open(Manipulator manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_manipulator = manipulator;
    openPosition = -1.47;
    addRequirements(m_manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double openSpeed = m_manipulator.pinchController.calculate(m_manipulator.getPosition(), openPosition);
    // m_manipulator.pinch(openSpeed);

    m_manipulator.pinch(0.2);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


}
