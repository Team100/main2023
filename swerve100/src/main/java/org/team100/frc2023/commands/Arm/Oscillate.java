package org.team100.frc2023.commands.Arm;

import org.team100.frc2023.subsystems.arm.ArmController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Oscillate extends CommandBase {
  /** Creates a new Oscillate. */
  ArmController m_arm;
  boolean movingUp = false;
  public Oscillate(ArmController arm) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(m_arm.getUpperArm() < m_arm.coneSubVal){
        movingUp = false;
    } else {
        movingUp = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_arm.getUpperArm() < m_arm.coneSubVal - 0.025 ){
        movingUp = true;
    }

    if(m_arm.getUpperArm() > m_arm.coneSubVal + 0.025){
        movingUp = false;
    }

    if(movingUp){
        m_arm.setUpperArm(0.1);
    } else {
        m_arm.setUpperArm(-0.15);
    }

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
