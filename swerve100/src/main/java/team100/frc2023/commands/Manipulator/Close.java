// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team100.frc2023.commands.Manipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team100.frc2023.subsystems.Manipulator;

public class Close extends CommandBase {
  /** Creates a new Close. */
  Manipulator m_manipulator;
  boolean first = true;
  Timer m_timer;
  public Close(Manipulator manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_manipulator = manipulator;
    m_timer = new Timer();
    addRequirements(m_manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulator.pinch.motor.configPeakCurrentLimit(45);

    m_timer.restart();
    first = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if(m_manipulator.getStatorCurrent() <= -11){
    //   // first = false;

    //   if(m_timer.get() >= 1){

    //     m_manipulator.pinch.motor.configPeakCurrentLimit(5);
    //     System.out.println("DEEZ");
    //   }
      

    // }

    // System.out.println("YOOOOO");

    if(m_manipulator.hasGamepiece() == false){
        m_manipulator.pinch(-0.8);
    } else {
        System.out.println("YESSS");
        m_manipulator.pinch.motor.configPeakCurrentLimit(7);
        m_manipulator.pinch(-0.2);

    }

    // m_manipulator.pinch(-0.8);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_manipulator.pinch(0);

    m_manipulator.pinch.motor.configPeakCurrentLimit(30);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
