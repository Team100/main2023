// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class Home extends CommandBase {
  /** Creates a new Home. */
  Manipulator m_manipulator;
  public Home(Manipulator manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_manipulator = manipulator;
    addRequirements(m_manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulator.pinch(0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_manipulator.getForwardLimitSwitch() == true){
        m_manipulator.pinch(0);
        m_manipulator.position.reset();
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
