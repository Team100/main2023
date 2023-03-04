// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;


public class CurrentFeedbackClose extends CommandBase {
  /** Creates a new CurrentFeedbackClose. */
  double closedCurrent;
  double force;
  boolean finishedFlag;
  double stepForce;
  //FRCTalonSRX pinch;

  private final Manipulator manip;


  public CurrentFeedbackClose(Manipulator subsystem, double closedCurrentParm, double forceParm) {
    manip = subsystem;
    closedCurrent = closedCurrentParm;
    force = forceParm;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finishedFlag = false;
    stepForce = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (manip.getStatorCurrent() <= closedCurrent) {
      stepForce = Math.min(stepForce + 0.07 * force, force);
      manip.pinch(stepForce);
    } else {
    manip.pinch (0); 
    finishedFlag = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finishedFlag) return true;
     else return false;
  }

}

