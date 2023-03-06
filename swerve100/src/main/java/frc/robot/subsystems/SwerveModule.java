package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule implements Sendable {
    private final String m_name;
    private final DriveMotor m_driveMotor;
    private final TurningMotor m_turningMotor;
    private final DriveEncoder m_driveEncoder;
    private final TurningEncoder m_turningEncoder;
    private final PIDController m_driveController;
    private final ProfiledPIDController m_turningController;
    private final SimpleMotorFeedforward m_turningFeedforward;
    private final SimpleMotorFeedforward m_driveFeedforward;

    public double m_tFeedForwardOutput;
    public double m_dFeedForwardOutput;
    public double turningMotorControllerOutput;
    public double driveMotorControllerOutput;

    // for calculating acceleration
    private double previousSpeedMetersPerSecond = 0;

    public SwerveModule(
            String name,
            DriveMotor driveMotor,
            TurningMotor turningMotor,
            DriveEncoder driveEncoder,
            TurningEncoder turningEncoder,
            PIDController driveController,
            ProfiledPIDController turningController,
            SimpleMotorFeedforward driveFeedforward,
            SimpleMotorFeedforward turningFeedforward) {
        m_name = name;
        m_driveMotor = driveMotor;
        m_turningMotor = turningMotor;
        m_driveEncoder = driveEncoder;
        m_turningEncoder = turningEncoder;
        m_driveController = driveController;
        m_turningController = turningController;
        m_driveFeedforward = driveFeedforward;
        m_turningFeedforward = turningFeedforward;
        SmartDashboard.putData(String.format("Swerve Module %s", m_name), this);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getAngle()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(
                desiredState,
                new Rotation2d(m_turningEncoder.getAngle()));
        driveMotorControllerOutput = m_driveController.calculate(
                m_driveEncoder.getRate(),
                state.speedMetersPerSecond);
        turningMotorControllerOutput = m_turningController.calculate(
                m_turningEncoder.getAngle(),
                state.angle.getRadians());
        m_tFeedForwardOutput = m_turningFeedforward.calculate(getTurnSetpointVelocity(), 0);
        double accelMetersPerSecondPerSecond = (state.speedMetersPerSecond - previousSpeedMetersPerSecond) / 0.02;
        previousSpeedMetersPerSecond = state.speedMetersPerSecond;
        m_dFeedForwardOutput = m_driveFeedforward.calculate(
                state.speedMetersPerSecond,
                accelMetersPerSecondPerSecond);
        setOutput(driveMotorControllerOutput + m_dFeedForwardOutput,
                turningMotorControllerOutput + m_tFeedForwardOutput);
    }

    // public void setOutput(double driveOutput, double turnOutput) {
    public void setOutput(double driveOutput, double turnOutput) {
        m_driveMotor.set(driveOutput);
        m_turningMotor.set(turnOutput);
    }

    public void resetEncoders() {
        m_driveEncoder.reset();
        m_turningEncoder.reset();
    }

    public void resetDriveEncoders() {
        m_driveEncoder.reset();
    }

    public double getTurnSetpointVelocity() {
        return m_turningController.getSetpoint().velocity;
    }

    public double getSetpointPosition() {
        return m_turningController.getSetpoint().position;
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
        return turningMotorControllerOutput;
    }

    public double getDControllerOutput() {
        return driveMotorControllerOutput;
    }

    public double driveSetpointMS() {
        return m_driveController.getSetpoint();
    }

    public double getdFeedForward() {
        return m_dFeedForwardOutput;
    }

    public double getGoal(){
        return m_turningController.getGoal().position;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(String.format("SwerveModule %s", m_name));
        builder.addDoubleProperty("speed_meters_per_sec", this::getSpeedMetersPerSecond, null);
        builder.addDoubleProperty("drive_motor_total_output", this::getDriveOutput, null);
        builder.addDoubleProperty("azimuth_degrees", this::getAzimuthDegrees, null);
        builder.addDoubleProperty("turning output Radians per second", this::getTurningOutput, null);
        builder.addDoubleProperty("Turning Setpoint velocity", this::getTurnSetpointVelocity, null);
        builder.addDoubleProperty("feed_forward_output", this::gettFeedForward, null);
        builder.addDoubleProperty("controller_Output_turning", this::getTControllerOutput, null);
        builder.addDoubleProperty("turningSetPoint", this::getSetpointPosition, null);

        builder.addDoubleProperty("Turning Goal", this::getGoal, null);

        builder.addDoubleProperty("driveControllerOutput", this::getDControllerOutput, null);
        builder.addDoubleProperty("driveSetPoint MS", this::driveSetpointMS, null);
        builder.addDoubleProperty("driveFeedForwardOutput", this::getdFeedForward, null);
        builder.addDoubleProperty("drive controller 'position' error which is really speed",
                () -> m_driveController.getPositionError(), null);
        builder.addDoubleProperty("drive controller 'velocity' error which is really accel",
                () -> m_driveController.getVelocityError(), null);
        builder.addDoubleProperty("turning controller position error", () -> m_turningController.getPositionError(),
                null);
        builder.addDoubleProperty("Drive Measurement MS", () -> m_driveEncoder.getRate(), null);
        builder.addDoubleProperty("Turning Measurement Angle Radians", () -> m_turningEncoder.getAngle(), null);

        // builder.addDoubleProperty("TURNING OUTPUT", () -> m_turningMotor, null );
    }
}
