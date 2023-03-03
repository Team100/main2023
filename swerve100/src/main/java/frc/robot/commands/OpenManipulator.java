// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class OpenManipulator extends CommandBase {
  private final Manipulator m_subsystem;
  private double closePosition;
  private Supplier<Boolean> shouldEnd = () -> false;

  public OpenManipulator(Manipulator subsystem, Supplier<Boolean> shouldEnd) {
    this(subsystem);
    this.shouldEnd = shouldEnd;
  }

  public OpenManipulator(Manipulator subsystem) {
    m_subsystem = subsystem;

   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    closePosition =200+ m_subsystem.getOrigin();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_subsystem.getEncoderPosition()>closePosition){
      m_subsystem.pinch(-0.2);
    } else {
      m_subsystem.pinch(0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.pinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shouldEnd.get()) return true;
    return Math.abs(closePosition-m_subsystem.getEncoderPosition()) < 50;
  }
}