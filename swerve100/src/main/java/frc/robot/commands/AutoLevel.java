// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoLevel extends CommandBase {
  private final SwerveDriveSubsystem drivetrain;
  private AHRS m_gyro;
  private int count = 0;
  boolean reversed;
    /** Creates a new autoLevel. */
  double startX = 0;
  public AutoLevel(boolean r, AHRS gyro, SwerveDriveSubsystem we) {
    drivetrain = we;
    m_gyro = gyro;
    reversed = r;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startX = drivetrain.getPose().getX();
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {

    if(reversed){
        if(drivetrain.getPose().getX() <= 4.155){
            double Roll = m_gyro.getRoll();
            double Pitch = m_gyro.getPitch();
                // System.out.println(Roll);
                double driveRollAmount = MathUtil.clamp(0.005 * Roll, -0.08, 0.08);
                double drivePitchAmount = MathUtil.clamp(0.005   * Pitch, -0.08, 0.08);
                System.out.println(drivePitchAmount);
        
               if(Math.abs(Roll) > 2.5 || Math.abs(Pitch) > 2.5){   
                count = 0;
                drivetrain.driveSlow(drivePitchAmount, -driveRollAmount, 0, false);     
               } else{
                count++;
               }
        } else {
            drivetrain.drive(-0.3, 0, 0, true);
        }
    } else {
        if(drivetrain.getPose().getX() >= 3.277){ //TODO real number needed TOTAL GUESS
            double Roll = m_gyro.getRoll();
            double Pitch = m_gyro.getPitch();
                // System.out.println(Roll);
                double driveRollAmount = MathUtil.clamp(0.004 * Roll, -0.08, 0.08);
                double drivePitchAmount = MathUtil.clamp(0.004   * Pitch, -0.08, 0.08);
                System.out.println(drivePitchAmount);
        
               if(Math.abs(Roll) > 2.5 || Math.abs(Pitch) > 2.5){   
                count = 0;
                drivetrain.driveSlow(drivePitchAmount, -driveRollAmount, 0, false);     
               } else{
                count++;
               }
        } else {
            drivetrain.drive(0.3, 0, 0, true);
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
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= 20;
  }
}
