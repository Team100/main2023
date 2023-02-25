// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveWithHeading extends CommandBase {
  /** Creates a new DrivePID. */
  SwerveDriveSubsystem m_robotDrive;
  PIDController m_headingController;
  Supplier<Rotation2d> m_desiredRotation;
  ChassisSpeeds targetChasisSpeeds = new ChassisSpeeds();

  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Pose2d currentPose;
//   Supplier<Double> rotSpeed;
  double radiantsPerSecond = 0;

  public DriveWithHeading(SwerveDriveSubsystem robotDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Rotation2d> desiredRotation, String name) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = robotDrive;
    m_headingController = m_robotDrive.headingController;
    m_desiredRotation = desiredRotation;

    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    // this.rotSpeed = rotSpeed;

    // m_headingController.setTolerance(0.1);
    m_headingController.enableContinuousInput(-Math.PI, Math.PI);
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
    double currentRads = MathUtil.angleModulus(currentPose.getRotation().getRadians());
    double desiredRads = MathUtil.angleModulus(m_desiredRotation.get().getRadians());
    radiantsPerSecond = m_headingController.calculate(currentRads, desiredRads);


    
    targetChasisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, radiantsPerSecond, currentPose.getRotation());

    

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
        builder.addDoubleProperty("ControllerOutput", () -> radiantsPerSecond, null);
        builder.addDoubleProperty("Chassis Speeds", () -> targetChasisSpeeds.omegaRadiansPerSecond, null);

    }
}
