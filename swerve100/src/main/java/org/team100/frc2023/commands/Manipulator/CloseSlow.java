package org.team100.frc2023.commands.Manipulator;

import org.team100.frc2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseSlow extends CommandBase {
  /** Creates a new CloseSlow. */

  Manipulator m_manipulator;
  public CloseSlow(Manipulator manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_manipulator = manipulator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulator.pinch.motor.configPeakCurrentLimit(10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulator.pinch(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_manipulator.pinch.motor.configPeakCurrentLimit(30);

    m_manipulator.pinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
