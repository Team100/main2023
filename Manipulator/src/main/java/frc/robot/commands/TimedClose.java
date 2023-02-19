// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class TimedClose extends CommandBase {
  /** Creates a new TimedClose. */
  int loopCount;
  int duration;
  double force;
  boolean finishedFlag;
  //duration is given in miliseconds and force is in between 0 and 1
 
    private final Manipulator manip;

   public TimedClose(Manipulator subsystem, int durationParm, double forceParm) {
    manip = subsystem;
    duration = durationParm;
    force = forceParm;
    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopCount = duration/20;
    finishedFlag = false;
    // System.out.println("we got to timedclosed");
    // System.out.println(duration);
    // System.out.println(force);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    manip.pinch(force);
    if (loopCount-- <= 0){
      finishedFlag = true;
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manip.pinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedFlag; 
  }
}
