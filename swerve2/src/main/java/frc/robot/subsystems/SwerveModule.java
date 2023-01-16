// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.motors.DriveEncoder;
import frc.robot.subsystems.motors.DriveMotor;
import frc.robot.subsystems.motors.TurningEncoder;
import frc.robot.subsystems.motors.TurningMotor;
import frc.robot.Constants.KModule;

public class SwerveModule implements Sendable {
    private final String m_name;
    private final DriveMotor m_driveMotor;
    private final TurningMotor m_turningMotor;

    private final DriveEncoder m_driveEncoder;
    private final TurningEncoder m_turningEncoder;

    private final PIDController m_drivePIDController;
    private final ProfiledPIDController m_turningPIDController;

    private final SimpleMotorFeedforward m_turningFeedforward;
    private final SimpleMotorFeedforward m_driveFeedforward;
    
    public double m_tFeedForwardOutput;
    public double m_dFeedForwardOutput;
    public double m_controllerOutput;
    public double m_driveOutput;

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

        m_drivePIDController = new PIDController(KModule.kPModuleDriveController, 0, 0);

        m_turningPIDController = new ProfiledPIDController(
            KModule.kPModuleTurningController,
                .1,
                0,
                new TrapezoidProfile.Constraints(
                        KModule.kMaxModuleAngularSpeedRadiansPerSecond,
                        KModule.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

        m_turningPIDController.enableContinuousInput(0, 2 * Math.PI);

        m_turningFeedforward = new SimpleMotorFeedforward(0.1, 0.001); // TODO: real values for kS and kV.
        SmartDashboard.putData(String.format("Swerve Module %s", m_name), this);
        m_driveFeedforward = new SimpleMotorFeedforward(0.0, .25);
        // TODO: extract this config
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getAngle()));
    }
    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getAngle()));
    }

    /**
     * Sets the desired state of the swerve module.
     * @param desiredState The desired state of the swerve module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getAngle()));

        // Calculate the drive output from the drive PID controller.
        m_driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        m_controllerOutput = m_turningPIDController.calculate(m_turningEncoder.getAngle(), state.angle.getRadians());

        m_tFeedForwardOutput = m_turningFeedforward.calculate(getSetpointVelocity(), 0);
        m_dFeedForwardOutput = m_driveFeedforward.calculate(getDriveSetpoint(), 0);
        setOutput(m_driveOutput + m_dFeedForwardOutput, m_controllerOutput + m_tFeedForwardOutput);
    }

    public void setOutput(double driveOutput, double turnOutput) {
        m_driveMotor.set(driveOutput);
        m_turningMotor.set(turnOutput);
    }

    public void resetEncoders() {
        m_driveEncoder.reset();
        m_turningEncoder.reset();
    }

    public double getSetpointVelocity() {
        return m_turningPIDController.getSetpoint().velocity;
    }

    public double getSetpointPosition() {
        return m_turningPIDController.getSetpoint().position;
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

    public double gettFeedForward() {
        return m_tFeedForwardOutput;
    }

    public double getTControllerOutput() {
        return m_controllerOutput;
    }

    public double getDControllerOutput() {
        return m_driveOutput;
    }

    public double getDriveSetpoint() {
        return m_drivePIDController.getSetpoint();
    }

    public double getdFeedForward() {
        return m_dFeedForwardOutput;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(String.format("SwerveModule %s", m_name));
        builder.addDoubleProperty("speed_meters_per_sec", this::getSpeedMetersPerSecond, null);
        builder.addDoubleProperty("drive_output", this::getDriveOutput, null);
        builder.addDoubleProperty("azimuth_degrees", this::getAzimuthDegrees, null);
        builder.addDoubleProperty("turning_output", this::getTurningOutput, null);
        builder.addDoubleProperty("setpoint velocity", this::getSetpointVelocity, null);
        builder.addDoubleProperty("feed_forward_output", this::gettFeedForward, null);
        builder.addDoubleProperty("controller_Output", this::getTControllerOutput, null);
        builder.addDoubleProperty("turningSetPoint", this::getSetpointPosition, null);
        builder.addDoubleProperty("driveControllerOutput", this::getDControllerOutput, null);
        builder.addDoubleProperty("driveSetPoint", this::getDriveSetpoint, null);
        builder.addDoubleProperty("driveFeedForwardOutput", this::getdFeedForward, null);
        builder.addDoubleProperty("drive controller position error", () -> m_drivePIDController.getPositionError(), null );
        builder.addDoubleProperty("drive controller velocity error", () -> m_drivePIDController.getVelocityError(), null );

    }
}
