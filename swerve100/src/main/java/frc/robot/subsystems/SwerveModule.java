package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveEncoder;
import frc.robot.subsystems.drive.DriveMotor;
import frc.robot.subsystems.turning.TurningEncoder;
import frc.robot.subsystems.turning.TurningMotor;

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

    // private final SimpleMotorFeedforward m_headingTurnFeedforward;
    private final SimpleMotorFeedforward m_headingDriveFeedforward;

    public double turningFeedForwardOutput;
    public double driveFeedForwardOutput;
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
            SimpleMotorFeedforward turningFeedforward,
            SimpleMotorFeedforward headingDriveFeedforward
            ) {
        m_name = name;
        m_driveMotor = driveMotor;
        m_turningMotor = turningMotor;
        m_driveEncoder = driveEncoder;
        m_turningEncoder = turningEncoder;
        m_driveController = driveController;
        m_turningController = turningController;
        m_driveFeedforward = driveFeedforward;
        m_turningFeedforward = turningFeedforward;
        m_headingDriveFeedforward = headingDriveFeedforward;
        // m_headingTurnFeedforward = headingTurnFeedforward;
        SmartDashboard.putData(String.format("Swerve Module %s", m_name), this);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveSpeedMS(), getTurningRotation());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getDistance(), getTurningRotation());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getTurningRotation());
        driveMotorControllerOutput = m_driveController.calculate(getDriveSpeedMS(), state.speedMetersPerSecond);
        turningMotorControllerOutput = m_turningController.calculate(getTurningAngleRad(), state.angle.getRadians());
        turningFeedForwardOutput = m_turningFeedforward.calculate(getTurnSetpointVelocityRadS(), 0);
        // TODO: smooth out accel
        double accelMetersPerSecondPerSecond = (state.speedMetersPerSecond - previousSpeedMetersPerSecond) / 0.02;
        previousSpeedMetersPerSecond = state.speedMetersPerSecond;
        driveFeedForwardOutput = m_driveFeedforward.calculate(
                state.speedMetersPerSecond,
                accelMetersPerSecondPerSecond);

        // if(MathUtil.applyDeadband(driveMotorControllerOutput, .01) == 0){
        //     driveMotorControllerOutput = 0;
        //     driveFeedForwardOutput = 0;
        // }

        // if(MathUtil.applyDeadband(turningMotorControllerOutput, .01) == 0){
        //     turningMotorControllerOutput = 0;
        //     turningFeedForwardOutput = 0;
        // }


        double driveOutput = MathUtil.applyDeadband(driveMotorControllerOutput + driveFeedForwardOutput, 0.03);
        double turnOutput = MathUtil.applyDeadband(turningMotorControllerOutput + turningFeedForwardOutput, 0.03);

        setOutput(driveOutput,
                turnOutput);
    }

    public void setDesiredDriveState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getTurningRotation());
        driveMotorControllerOutput = m_driveController.calculate(getDriveSpeedMS(), state.speedMetersPerSecond);
        turningMotorControllerOutput = m_turningController.calculate(getTurningAngleRad(), state.angle.getRadians());
        turningFeedForwardOutput = m_turningFeedforward.calculate(getTurnSetpointVelocityRadS(), 0);
        // TODO: smooth out accel
        double accelMetersPerSecondPerSecond = (state.speedMetersPerSecond - previousSpeedMetersPerSecond) / 0.02;
        previousSpeedMetersPerSecond = state.speedMetersPerSecond;
        driveFeedForwardOutput = m_headingDriveFeedforward.calculate(
                state.speedMetersPerSecond,
                accelMetersPerSecondPerSecond);

        // if(MathUtil.applyDeadband(driveMotorControllerOutput, .01) == 0){
        //     driveMotorControllerOutput = 0;
        //     driveFeedForwardOutput = 0;
        // }

        // if(MathUtil.applyDeadband(turningMotorControllerOutput, .01) == 0){
        //     turningMotorControllerOutput = 0;
        //     turningFeedForwardOutput = 0;
        // }


        double driveOutput = MathUtil.applyDeadband(driveMotorControllerOutput + driveFeedForwardOutput, 0.03);
        double turnOutput = MathUtil.applyDeadband(turningMotorControllerOutput + turningFeedForwardOutput, 0.03);

        setOutput(driveOutput,
                turnOutput);
    }

    public void setDesiredStateNoFF(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getTurningRotation());
        driveMotorControllerOutput = m_driveController.calculate(getDriveSpeedMS(), state.speedMetersPerSecond);
        turningMotorControllerOutput = m_turningController.calculate(getTurningAngleRad(), state.angle.getRadians());
        turningFeedForwardOutput = m_turningFeedforward.calculate(getTurnSetpointVelocityRadS(), 0);
        // TODO: smooth out accel
        double accelMetersPerSecondPerSecond = (state.speedMetersPerSecond - previousSpeedMetersPerSecond) / 0.02;
        previousSpeedMetersPerSecond = state.speedMetersPerSecond;
        driveFeedForwardOutput = m_driveFeedforward.calculate(
                state.speedMetersPerSecond,
                accelMetersPerSecondPerSecond);

        setOutput(driveMotorControllerOutput,
                turningMotorControllerOutput + turningFeedForwardOutput);
    }

    /**
     * @param driveOutput in range [-1, 1]
     * @param turnOutput  in range [-1, 1]
     */
    public void setOutput(double driveOutput, double turnOutput) {
        m_driveMotor.set(driveOutput);
        m_turningMotor.set(turnOutput);
    }

    /** Reset distance and angle to zero. */
    public void resetEncoders() {
        m_driveEncoder.reset();
        m_turningEncoder.reset();
    }

    /** Reset just distance to zero, leave angle alone. */
    public void resetDriveEncoders() {
        m_driveEncoder.reset();
    }

    private double getDriveSpeedMS() {
        return m_driveEncoder.getRate();
    }

    private double getTurnSetpointVelocityRadS() {
        return m_turningController.getSetpoint().velocity;
    }

    private double getTurningAngleRad() {
        return m_turningEncoder.getAngle();
    }

    private Rotation2d getTurningRotation() {
        return new Rotation2d(getTurningAngleRad());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(String.format("SwerveModule %s", m_name));
        // Measurements
        builder.addDoubleProperty("Drive position (m)", () -> getPosition().distanceMeters, null);
        builder.addDoubleProperty("Drive Speed (m/s)", () -> getDriveSpeedMS(), null);
        builder.addDoubleProperty("Turning Angle (rad)", () -> getTurningAngleRad(), null);
        builder.addDoubleProperty("Turning Angle (deg)", () -> Units.radiansToDegrees(getTurningAngleRad()), null);

        // Turning
        builder.addDoubleProperty("Turning Goal (rad)", () -> m_turningController.getGoal().position, null);
        builder.addDoubleProperty("Turning Setpoint (rad)", () -> m_turningController.getSetpoint().position, null);
        builder.addDoubleProperty("Turning Setpoint Velocity (rad/s)", this::getTurnSetpointVelocityRadS, null);
        builder.addDoubleProperty("Turning Position Error (rad)", () -> m_turningController.getPositionError(), null);
        builder.addDoubleProperty("Turning Velocity Error (rad/s)", () -> m_turningController.getVelocityError(), null);
        builder.addDoubleProperty("Turning Controller Output", () -> turningMotorControllerOutput, null);
        builder.addDoubleProperty("Turning Feed Forward Output", () -> turningFeedForwardOutput, null);
        builder.addDoubleProperty("Turning Motor Output [-1, 1]", () -> m_turningMotor.get(), null);

        // Drive
        builder.addDoubleProperty("Drive Setpoint (m/s)", () -> m_driveController.getSetpoint(), null);
        builder.addDoubleProperty("Drive Speed Error (m/s)", () -> m_driveController.getPositionError(), null);
        builder.addDoubleProperty("Drive Accel Error (m/s/s)", () -> m_driveController.getVelocityError(), null);
        builder.addDoubleProperty("Drive Controller Output", () -> driveMotorControllerOutput, null);
        builder.addDoubleProperty("Drive Feed Forward Output", () -> driveFeedForwardOutput, null);
        builder.addDoubleProperty("Drive Motor Output [-1, 1]", () -> m_driveMotor.get(), null);

    }
}
