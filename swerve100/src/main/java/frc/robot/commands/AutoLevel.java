// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AHRSClass;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoLevel extends CommandBase {
  private final AHRSClass m_gyro;
  private final SwerveDriveSubsystem m_robotDrive;
  private int count = 0;
  boolean reversed;
    /** Creates a new autoLevel. */
  double startX = 0;
  public AutoLevel(boolean r, SwerveDriveSubsystem robotDrive, AHRSClass gyro) {
    m_gyro = gyro;
    m_robotDrive = robotDrive;
    reversed = r;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startX = m_robotDrive.getPose().getX();
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled
  public void execute() {

    if(reversed){
        // if(m_robotDrive.getPose().getX() <= 4.354773){ //4.125
            double Roll = m_gyro.getRedundantRoll();    //
            double Pitch = m_gyro.getRedundantPitch();
                // System.out.println(Roll);
                double driveRollAmount = MathUtil.clamp(0.005 * Roll, -0.08, 0.08);
                double drivePitchAmount = MathUtil.clamp(0.005   * Pitch, -0.08, 0.08);
                System.out.println(drivePitchAmount);
        
               if(Math.abs(Roll) > 2.5 || Math.abs(Pitch) > 2.5){   //
                count = 0;
                // m_robotDrive.driveSlow(drivePitchAmount, driveRollAmount, 0, true);     
                m_robotDrive.driveSlow(drivePitchAmount, driveRollAmount, 0, false);   //was false be
                System.out.println(drivePitchAmount);

               } else{
                count++;
               }
        // } else {
            // m_robotDrive.drive(-0.2, 0, 0, true);
        // }
    } else {
        if(m_robotDrive.getPose().getX() >= 3.277){ //TODO real number needed TOTAL GUESS
            double Roll = m_gyro.getRedundantRoll();
            double Pitch = m_gyro.getRedundantPitch();
                // System.out.println(Roll);
                double driveRollAmount = MathUtil.clamp(0.004 * Roll, -0.08, 0.08);
                double drivePitchAmount = MathUtil.clamp(0.004   * Pitch, -0.08, 0.08);
                System.out.println(drivePitchAmount);
        
               if(Math.abs(Roll) > 2.5 || Math.abs(Pitch) > 2.5){   
                count = 0;
                m_robotDrive.driveSlow(drivePitchAmount, -driveRollAmount, 0, false);     
               } else{
                count++;
               }
        } else {
            m_robotDrive.drive(0.3, 0, 0, true);
        }
    }


    // double Roll = m_gyro.getRoll();
    //         double Pitch = m_gyro.getPitch();
    //             // System.out.println(Roll);
    //             double driveRollAmount = MathUtil.clamp(0.004 * Roll, -0.08, 0.08);
    //             double drivePitchAmount = MathUtil.clamp(0.004   * Pitch, -0.08, 0.08);
    //             System.out.println(drivePitchAmount);
        
    //            if(Math.abs(Roll) > 2.5 || Math.abs(Pitch) > 2.5){   
    //             count = 0;
    //             drivetrain.drive(drivePitchAmount, -0, 0, false);     
    //            } else{
    //             count++;
    //            }

   

    
    }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AUTO LEVEL ENDING");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= 20;
  }
}
