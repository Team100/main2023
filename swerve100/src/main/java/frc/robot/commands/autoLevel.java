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
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    double Roll = m_gyro.getRoll();
    double Pitch = m_gyro.getPitch();
    // System.out.println(Roll);
        double driveRollAmount = MathUtil.clamp(0.005 * Roll, -0.06, 0.06);
        double drivePitchAmount = MathUtil.clamp(0.005   * Pitch, -0.06, 0.06);
       if(Math.abs(Roll) > 2.5 || Math.abs(Pitch) > 2.5){   
        drivetrain.drive(driveRollAmount, drivePitchAmount, 0, false);     
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
