// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  public static final int kFrontLeftDriveMotorPort = 0;
  public static final int kRearLeftDriveMotorPort = 1;
  public static final int kFrontRightDriveMotorPort = 15;
  public static final int kRearRightDriveMotorPort = 14;

  public static final int kFrontLeftTurningMotorPort = 4;
  public static final int kFrontLeftAngleRange = 893;
  public static final int kFrontLeftAngleZero = 770;

  public static final int kRearLeftTurningMotorPort = 5;
  public static final int kRearLeftAngleRange = 880;
  public static final int kRearLeftAngleZero = 870;

  public static final int kFrontRightTurningMotorPort = 11;
  public static final int kFrontRightAngleRange = 893;
  public static final int kFrontRightAngleZero = 675;

  public static final int kRearRightTurningMotorPort = 10;
  public static final int kRearRightAngleRange = 886;
  public static final int kRearRightAngleZero = 200;

  public static final boolean kFrontLeftTurningEncoderReversed = false;
  public static final boolean kRearLeftTurningEncoderReversed = true;
  public static final boolean kFrontRightTurningEncoderReversed = false;
  public static final boolean kRearRightTurningEncoderReversed = true;

  public static final boolean kFrontLeftDriveEncoderReversed = true;
  public static final boolean kRearLeftDriveEncoderReversed = true;
  public static final boolean kFrontRightDriveEncoderReversed =true;
  public static final boolean kRearRightDriveEncoderReversed = true;

  public static final double kTrackWidth = 0.5;
  // Distance between centers of right and left wheels on robot
  public static final double kWheelBase = 0.7;
  // Distance between front and back wheels on robot
  public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  public static final boolean kGyroReversed = false;

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or theoretically
  // for *your* robot's drive.
  // The SysId tool provides a convenient method for obtaining these values for your robot.
  // TODO: add some feedforward.
  public static final double ksVolts = 2;
  public static final double kvVoltSecondsPerMeter = 2.0;
  public static final double kaVoltSecondsSquaredPerMeter = 0.5;

  public static final double kMaxSpeedMetersPerSecond = 2;
  
  // Robot swerve modules
  private final SwerveModule m_frontLeft = SwerveModuleFactory.newSwerveModule(
          "Front Left",
          kFrontLeftDriveMotorPort,
          kFrontLeftTurningMotorPort,
          kFrontLeftDriveEncoderReversed,
          kFrontLeftTurningEncoderReversed,
          kFrontLeftAngleRange,
          kFrontLeftAngleZero);

  private final SwerveModule m_rearLeft = SwerveModuleFactory.newSwerveModule(
          "Rear Left",
          kRearLeftDriveMotorPort,
          kRearLeftTurningMotorPort,
          kRearLeftDriveEncoderReversed,
          kRearLeftTurningEncoderReversed,
          kRearLeftAngleRange,
          kRearLeftAngleZero);

  private final SwerveModule m_frontRight = SwerveModuleFactory.newSwerveModule(
          "Front Right",
          kFrontRightDriveMotorPort,
          kFrontRightTurningMotorPort,
          kFrontRightDriveEncoderReversed,
          kFrontRightTurningEncoderReversed,
          kFrontRightAngleRange,
          kFrontRightAngleZero);

  private final SwerveModule m_rearRight = SwerveModuleFactory.newSwerveModule(
          "Rear Right",
          kRearRightDriveMotorPort,
          kRearRightTurningMotorPort,
          kRearRightDriveEncoderReversed,
          kRearRightTurningEncoderReversed,
          kRearRightAngleRange,
          kRearRightAngleZero);

  // The gyro sensor.  We have a Nav-X.
  private final AHRS m_gyro;
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;
      
  public DriveSubsystem() {
    m_gyro = new AHRS(SerialPort.Port.kUSB);
    m_gyro.reset();
    m_odometry = new SwerveDriveOdometry(kDriveKinematics, Rotation2d.fromDegrees(-m_gyro.getFusedHeading()));
    SmartDashboard.putData("Drive Subsystem", this);
  }

  @Override
  public void periodic() {
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getFusedHeading()),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(-m_gyro.getFusedHeading()));
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
    if (Math.abs(xSpeed)<.1) xSpeed=0;
    if (Math.abs(ySpeed)<.1) ySpeed=0;
    if (Math.abs(rot)<.1) rot=0;
    var swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(kMaxSpeedMetersPerSecond*xSpeed, kMaxSpeedMetersPerSecond*ySpeed, rot, Rotation2d.fromDegrees(-m_gyro.getFusedHeading()))
                : new ChassisSpeeds(kMaxSpeedMetersPerSecond*xSpeed, kMaxSpeedMetersPerSecond*ySpeed, 5*rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(new SwerveModuleState(-swerveModuleStates[0].speedMetersPerSecond, swerveModuleStates[0].angle));
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(new SwerveModuleState(-swerveModuleStates[2].speedMetersPerSecond, swerveModuleStates[2].angle));
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond);
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

  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getFusedHeading()).getDegrees();
  }

  public double getTurnRate() {
    return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("heading_degrees", this::getHeading, null);
  }
}