// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import ctre_shims.TalonAngle;
import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule implements Sendable {
  private final WPI_TalonSRX m_driveMotor;
  private final WPI_TalonSRX m_turningMotor;

  private final TalonEncoder m_driveEncoder;
  private final TalonAngle m_turningEncoder;

  private TalonSRXSimCollection m_driveSim;
  private TalonSRXSimCollection m_turningSim;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 1);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      int angleRange,
      int angleZero) {
    m_driveMotor = new WPI_TalonSRX(driveMotorChannel);
    m_driveMotor.configFactoryDefault();
    m_driveMotor.configSupplyCurrentLimit(
      new SupplyCurrentLimitConfiguration(true, ModuleConstants.kDriveCurrentLimit, 0, 0));
    // I think we're using the AndyMark CIMcoder, either am-3314a.  TODO: verify.
    m_driveMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 0);
    m_driveMotor.configFeedbackNotContinuous(false, 0); // just keep counting ticks
    m_driveMotor.setSelectedSensorPosition(0); // TODO: is this required?

    m_turningMotor = new WPI_TalonSRX(turningMotorChannel);
    m_turningMotor.configFactoryDefault();
    m_turningMotor.configSupplyCurrentLimit(
      new SupplyCurrentLimitConfiguration(true, ModuleConstants.kTurningCurrentLimit, 0, 0));
    // I think we're using the AndyMark MA3, am-2899, on the steering shaft.  TODO: verify.
    m_turningMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0);
    m_turningMotor.configFeedbackNotContinuous(true, 0); // just return the angle

    m_driveEncoder = new TalonEncoder(m_driveMotor);
    m_turningEncoder = new TalonAngle(m_turningMotor);

    m_driveSim = m_driveMotor.getSimCollection();
    m_turningSim = m_turningMotor.getSimCollection();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not
    m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_turningEncoder.setInputRange(angleRange);
    m_turningEncoder.setInputOffset(angleZero);

    // Set whether turning encoder should be reversed or not
    m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // getDistance == radians, i.e. ticks * distance per tick.
    // TODO: this eliminates wrapping which is not necessary; use getAnalogInRaw instead
    return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

    setOutput(driveOutput, turnOutput);
  }

  public void setOutput(double driveOutput, double turnOutput) {
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.reset();
    m_turningEncoder.reset();
  }

  public TalonEncoder getDriveEncoder() {
    return m_driveEncoder;
  }

  public TalonAngle getTurningEncoder() {
    return m_turningEncoder;
  }

  public double getAzimuthDegrees() {
    return new Rotation2d(m_turningEncoder.getDistance()).getDegrees();
  }

  public double getTurningPosition() {
    return m_turningEncoder.get();
  }

  public double getTurningAnalogIn() {
    return m_turningEncoder.getAnalogIn();
  }

  public double getTurningAnalogInRaw() {
    return m_turningEncoder.getAnalogInRaw();
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
    builder.setSmartDashboardType("SwerveModule");
    builder.addDoubleProperty("speed_meters_per_sec", this::getSpeedMetersPerSecond, null);
    builder.addDoubleProperty("drive_output", this::getDriveOutput, null);
    builder.addDoubleProperty("azimuth_degrees", this::getAzimuthDegrees, null);
    builder.addDoubleProperty("turning_output", this::getTurningOutput, null);
    builder.addDoubleProperty("turning_position", this::getTurningPosition, null);
    builder.addDoubleProperty("turning_analog_in", this::getTurningAnalogIn, null);
    builder.addDoubleProperty("turning_analog_in_raw", this::getTurningAnalogInRaw, null);
  }

  public void simulationPeriodic(double dt) {
    // see CTRE phoenix examples DifferentialDrive_Simulation
    m_driveSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_turningSim.setBusVoltage(RobotController.getBatteryVoltage());

    m_driveSim.setQuadratureRawPosition(0); // TODO: math
    m_driveSim.setQuadratureVelocity(0); // TODO: math
    m_turningSim.setAnalogPosition(0); // TODO: math
    m_turningSim.setAnalogVelocity(0); // TODO: math


  }
}
