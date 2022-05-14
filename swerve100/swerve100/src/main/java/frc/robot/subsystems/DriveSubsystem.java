// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed);

  // The gyro sensor.  We have a navx.
  private final Gyro m_gyro = new AHRS(SerialPort.Port.kUSB);

  // These are for simulation
  private final Field2d m_fieldSim = new Field2d();
  private double m_yawValue;
  private SimDouble m_angleSim;


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // for simulation, see pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java
    SmartDashboard.putData("Field", m_fieldSim);
    SmartDashboard.putData("Front Left Module", m_frontLeft);
    SmartDashboard.putData("Front Right Module", m_frontRight);
    SmartDashboard.putData("Rear Left Module", m_rearLeft);
    SmartDashboard.putData("Rear Right Module", m_rearRight);
    SmartDashboard.putData("Front Left Turn Encoder", m_frontLeft.getTurningEncoder());
    SmartDashboard.putData("Front Right Turn Encoder", m_frontRight.getTurningEncoder());
    SmartDashboard.putData("Rear Left Turn Encoder", m_rearLeft.getTurningEncoder());
    SmartDashboard.putData("Rear Right Turn Encoder", m_rearRight.getTurningEncoder());
    SmartDashboard.putData("Front Left Drive Encoder", m_frontLeft.getDriveEncoder());
    SmartDashboard.putData("Front Right Drive Encoder", m_frontRight.getDriveEncoder());
    SmartDashboard.putData("Rear Left Drive Encoder", m_rearLeft.getDriveEncoder());
    SmartDashboard.putData("Rear Right Drive Encoder", m_rearRight.getDriveEncoder());
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    m_angleSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void test(double[][] desiredOutputs) {
    System.out.println("set outputs");
    m_frontLeft.setOutput(desiredOutputs[0][0], desiredOutputs[0][1]);
    m_frontRight.setOutput(desiredOutputs[1][0], desiredOutputs[1][1]);
    m_rearLeft.setOutput(desiredOutputs[2][0], desiredOutputs[2][1]);
    m_rearRight.setOutput(desiredOutputs[3][0], desiredOutputs[3][1]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("heading_degrees", this::getHeading, null);
  }

  // see github.com/4201VitruvianBots/2021SwerveSim
  @Override
  public void simulationPeriodic() {
    m_frontLeft.simulationPeriodic(0.02);
    m_frontRight.simulationPeriodic(0.02);
    m_rearLeft.simulationPeriodic(0.02);
    m_rearRight.simulationPeriodic(0.02);

    SwerveModuleState[] moduleStates = {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };

    ChassisSpeeds chassisSpeed = DriveConstants.kDriveKinematics.toChassisSpeeds(moduleStates);
    double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;
    m_yawValue += chassisRotationSpeed * 0.02;
    m_angleSim.set(m_yawValue);
  }
}
