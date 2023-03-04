// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class Calibrate extends CommandBase {
  private final Manipulator m_subsystem;
private boolean foundInnerLimitSwitch = false;
private boolean foundOuterLimitSwitch = false;
private double innerSoftLimit;
private double outerSoftLimit;
  /** Creates a new Calibrate. */
  public Calibrate(Manipulator m) {
    m_subsystem=m;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.pinch(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if(!foundInnerLimitSwitch) {
      // System.out.println("Looking for Inner Switch...");
      if(m_subsystem.getInnerLimitSwitch()){
        System.out.println("Found Inner Switch!");
        m_subsystem.pinch(0);
        innerSoftLimit=m_subsystem.getEncoderPosition();
        foundInnerLimitSwitch=true;
        m_subsystem.pinch(-0.2);
      }
    }
    else if (!foundOuterLimitSwitch){
      if(m_subsystem.getOuterLimitSwitch()){
        m_subsystem.pinch(0);
        outerSoftLimit=m_subsystem.getEncoderPosition();
        foundOuterLimitSwitch=true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.configSoftLimits(innerSoftLimit, outerSoftLimit);
    System.out.println("inner: " + innerSoftLimit + ", outer: " + outerSoftLimit);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return foundInnerLimitSwitch && foundOuterLimitSwitch;
  }
}
