// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule implements Sendable {
  public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
  public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

  public static final int kCIMcoderEncoderCPR = 80; // see docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html

  public static final double kWheelDiameterMeters = 0.1016; // AndyMark Swerve & Steer has 4 inch wheel
  public static final double kDriveReduction = 6.67; // see andymark.com/products/swerve-and-steer
  public static final double kDriveEncoderDistancePerPulse =
      // Assumes the encoders are directly mounted on the wheel shafts
      (kWheelDiameterMeters * Math.PI) / ((double) kCIMcoderEncoderCPR * kDriveReduction);

  public static final double kPModuleTurningController = 0.001;

  public static final double kPModuleDriveController = 0.001;

  private final String m_name;
  private final DriveMotor m_driveMotor;
  private final TurningMotor m_turningMotor;

  private final DriveEncoder m_driveEncoder;
  private final TurningEncoder m_turningEncoder;

  private final PIDController m_drivePIDController;
  private final ProfiledPIDController m_turningPIDController;

  public SwerveModule(
      String name, 
      DriveMotor driveMotor,
      TurningMotor turningMotor,
      DriveEncoder driveEncoder,
      TurningEncoder turningEncoder) {
    m_name = name;
    m_driveMotor = driveMotor;
    m_turningMotor = turningMotor;
    m_driveEncoder = driveEncoder;
    m_turningEncoder = turningEncoder;

    m_drivePIDController = new PIDController(kPModuleDriveController, 0, 1);

    m_turningPIDController = new ProfiledPIDController(
          kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kMaxModuleAngularSpeedRadiansPerSecond,
              kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    SmartDashboard.putData(String.format("Swerve Module %s", m_name), this);
  }

  public SwerveModuleState getState() {
    // TODO: this eliminates wrapping which is not necessary; use getAnalogInRaw instead
    return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getAngle()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getAngle()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getAngle(), state.angle.getRadians());

    setOutput(driveOutput, turnOutput);
  }

  public void setOutput(double driveOutput, double turnOutput) {
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  public void resetEncoders() {
    m_driveEncoder.reset();
    m_turningEncoder.reset();
  }

  public DriveEncoder getDriveEncoder() {
    return m_driveEncoder;
  }

  public TurningEncoder getTurningEncoder() {
    return m_turningEncoder;
  }

  public double getAzimuthDegrees() {
    return new Rotation2d(m_turningEncoder.getAngle()).getDegrees();
  }

  public double getSpeedMetersPerSecond() {
    return m_driveEncoder.getRate();
  }

  public double getDriveOutput() {
    return m_driveMotor.get();
  }

  public double getTurningOutput() {
    return m_turningMotor.get();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(String.format("SwerveModule %s", m_name));
    builder.addDoubleProperty("speed_meters_per_sec", this::getSpeedMetersPerSecond, null);
    builder.addDoubleProperty("drive_output", this::getDriveOutput, null);
    builder.addDoubleProperty("azimuth_degrees", this::getAzimuthDegrees, null);
    builder.addDoubleProperty("turning_output", this::getTurningOutput, null);
  }
}
