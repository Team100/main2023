// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import team100.control.*;

import java.util.function.Supplier;

import javax.naming.ContextNotEmptyException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveModule;

public class DriveWithHeading extends CommandBase {
  /** Creates a new DrivePID. */
  SwerveDriveSubsystem m_robotDrive;
  PIDController m_headingController;
  Rotation2d m_desiredRotation;

  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Pose2d currentPose;
//   Supplier<Double> rotSpeed;

  public DriveWithHeading(SwerveDriveSubsystem robotDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Rotation2d desiredRotation, String name) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = robotDrive;
    m_headingController = m_robotDrive.headingController;
    m_desiredRotation = desiredRotation;

    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    // this.rotSpeed = rotSpeed;

    // m_headingController.setTolerance(0.1);
    // m_headingController.enableContinuousInput(-Math.PI, Math.PI);
    currentPose = m_robotDrive.getPose();

    SmartDashboard.putData("Drive with Heading: " + name, this);

    addRequirements(m_robotDrive); 

    



  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double x = xSpeed.get();
    double y = ySpeed.get();
    // double rot = rotSpeed.get();

    if(Math.abs(x) < 0.1){
        x = 0;
    }

    if(Math.abs(y) < 0.1){
        y = 0;
    }

    // if(Math.abs(rot) < 0.1){
    //     rot = 0;
    // }

    currentPose = m_robotDrive.getPose();
    double currentRads = currentPose.getRotation().getRadians();
    double desiredRads = m_desiredRotation.getRadians();
    double thetaOuput = 0;

    if (Math.abs(currentRads - desiredRads) >= 0.1) {
        thetaOuput = m_headingController.calculate(currentRads, desiredRads);
    }

    
    ChassisSpeeds targetChasisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, thetaOuput, currentPose.getRotation());

    SwerveModuleState[] targetModuleStates = SwerveDriveSubsystem.kDriveKinematics.toSwerveModuleStates(targetChasisSpeeds);

    m_robotDrive.setModuleStates(targetModuleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ENDINNGGGGGGGG");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Error", () -> m_headingController.getPositionError(), null);
        builder.addDoubleProperty("Measurment", () -> currentPose.getRotation().getRadians(), null);
        builder.addDoubleProperty("Setpoint", () ->  m_headingController.getSetpoint(), null);



    }
}
