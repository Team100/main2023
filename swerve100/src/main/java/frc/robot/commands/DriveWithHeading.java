// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveWithHeading extends CommandBase{
  /** Creates a new DrivePID. */
  SwerveDriveSubsystem m_robotDrive;
  ProfiledPIDController m_headingController;
  Supplier<Rotation2d> m_desiredRotation;
  //ChassisSpeeds targetChasisSpeeds = new ChassisSpeeds();

  private final DoubleSupplier xSpeed;
  private final DoubleSupplier ySpeed;
  private static final double kSpeedModifier = 1.0;
  Pose2d currentPose;
//   Supplier<Double> rotSpeed;
  double radiansPerSecond = 0;
  double yOutput;

  public DriveWithHeading(SwerveDriveSubsystem robotDrive, DoubleSupplier xSpeed, DoubleSupplier ySpeed, Supplier<Rotation2d> desiredRotation, String name) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = robotDrive;
    m_headingController = m_robotDrive.headingController;
    m_desiredRotation = desiredRotation;
    

    yOutput = 0;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    // this.rotSpeed = rotSpeed;

    // m_headingController.setTolerance(0.1);
    m_headingController.enableContinuousInput(-Math.PI, Math.PI);
    currentPose = m_robotDrive.getPose();

    SmartDashboard.putData("Drive with Heading:", this);

    addRequirements(m_robotDrive); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("aaaaaaaaaaaaaaaaaaaaaaaaaaaa");

    // if(Math.abs(rot) < 0.1){
    //     rot = 0;
    // }

    currentPose = m_robotDrive.getPose();
    double currentRads = MathUtil.angleModulus(currentPose.getRotation().getRadians());
    double desiredRads = MathUtil.angleModulus(m_desiredRotation.get().getRadians());
    radiansPerSecond = m_headingController.calculate(currentRads, desiredRads);


    double xSwitch = MathUtil.applyDeadband(xSpeed.getAsDouble(), .1);
    double ySwitch = MathUtil.applyDeadband(ySpeed.getAsDouble(), .1);


    // if(ySwitch == 0){
    //     yOutput = m_robotDrive.yController.calculate()
    // }

    double xDBRemoved = (xSwitch-.1*Math.signum(xSwitch))/.9;
    double yDBRemoved = (ySwitch-.1*Math.signum(ySwitch))/.9;
    
    // targetChasisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xDBRemoved * xDBRemoved * xDBRemoved * kSpeedModifier,
    // yDBRemoved * yDBRemoved * yDBRemoved * kSpeedModifier, radiantsPerSecond, currentPose.getRotation());

    

    // SwerveModuleState[] targetModuleStates = SwerveDriveSubsystem.kDriveKinematics.toSwerveModuleStates(targetChasisSpeeds);

    // m_robotDrive.setModuleStates(targetModuleStates);
    m_robotDrive.drive(xDBRemoved * xDBRemoved * xDBRemoved * kSpeedModifier,
    yDBRemoved * yDBRemoved * yDBRemoved * kSpeedModifier, radiansPerSecond, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
        builder.addDoubleProperty("Setpoint", () ->  m_headingController.getSetpoint().position, null);
        builder.addDoubleProperty("Goal", () ->  m_headingController.getGoal().position, null);
        builder.addDoubleProperty("ControllerOutput", () -> radiansPerSecond, null);
       // builder.addDoubleProperty("Chassis Speeds", () -> targetChasisSpeeds.omegaRadiansPerSecond, null);

    }
}
