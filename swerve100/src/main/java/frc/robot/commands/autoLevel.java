// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class autoLevel extends CommandBase {
  private final SwerveDriveSubsystem drivetrain;
  private AHRS m_gyro;
  double previousPitch = 0;
  int revCount;
  boolean isPitchMode;
  boolean backwards;
    /** Creates a new autoLevel. */
  public autoLevel(AHRS gyro, SwerveDriveSubsystem we) {
    drivetrain = we;
    m_gyro = gyro;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // previousPitch = m_gyro.getPitch()- 2.2;
    // revCount = 25;
    // isPitchMode = true;
    // previousPitch = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    double Roll = m_gyro.getRoll();
    double Pitch = m_gyro.getPitch();
    // System.out.println(Roll);
        double driveRollAmount = MathUtil.clamp(0.0045 * Roll, -0.06, 0.06);
        double drivePitchAmount = MathUtil.clamp(0.0045 * Pitch, -0.06, 0.06);
       if(Math.abs(Roll) > 2.5 || Math.abs(Pitch) > 2.5){   
        drivetrain.drive(driveRollAmount, drivePitchAmount, 0, false);     
       }
      
    /*double Pitch = m_gyro.getPitch() - 3.4;
    double changeInPitch = Pitch - previousPitch;
    previousPitch = Pitch;
    double driveAmount = MathUtil.clamp(0.5 * Pitch, -1, 1);
    System.out.println(changeInPitch);
    System.out.println(isPitchMode);
    System.out.println(m_gyro.getPitch());
    System.out.println(revCount);
    if (revCount != 0) {
      if (isPitchMode == true && changeInPitch > 1){
        isPitchMode = false;
        backwards = false;
      } else if(isPitchMode == true && changeInPitch < -1) {
        isPitchMode = false;
        backwards = true;
      } else if(isPitchMode == false && backwards == true) {
        drivetrain.driveVelocity(-0.12, -0.12);
        revCount = revCount - 1;
      } else if (isPitchMode == false && backwards == false) {
        drivetrain.driveVelocity(.12, .12);
        revCount = revCount - 1;
        } else if (Pitch > 2 && isPitchMode) {
            drivetrain.driveVelocity(driveAmount, driveAmount);
           } else if (Pitch < -2 && isPitchMode){
            drivetrain.driveVelocity(driveAmount, driveAmount);
            } 
      }*/
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
