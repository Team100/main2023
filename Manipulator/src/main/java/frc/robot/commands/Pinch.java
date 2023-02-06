package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class Pinch extends CommandBase { 

    private final Manipulator m_subsystem;

    
    public Pinch (Manipulator subsystem) {
         m_subsystem = subsystem;


  }

  @Override
  public void initialize(){
    m_subsystem.pinch(1);
  }

  @Override
  public boolean isFinished() {
    
    if (m_subsystem.getEncoderPosition()<=0){
        return true;
    }
    return false;
  } 

  @Override
  public void end(boolean interrupted){
    m_subsystem.pinch(0);
  }
}
