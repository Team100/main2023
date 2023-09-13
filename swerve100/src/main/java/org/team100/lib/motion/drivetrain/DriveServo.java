package org.team100.lib.motion.drivetrain;

import org.team100.lib.encoder.drive.DriveEncoder;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.drive.DriveMotor;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Feedforward and feedback control of a single drive motor. */
public class DriveServo implements Sendable {
    public static class Config {
        public double kDriveDeadband = 0.03;
    }

    private final Config m_config = new Config();
    private final Experiments m_experiments;
    private final DriveMotor m_driveMotor;
    private final DriveEncoder m_driveEncoder;
    private final PIDController m_driveController;
    private final SimpleMotorFeedforward m_driveFeedforward;

    public double driveFeedForwardOutput;
    public double driveMotorControllerOutput;

    // for calculating acceleration
    private double previousSpeedM_S = 0;

    private double m_driveSpeed;

    public DriveServo(
            Experiments experiments,
            String name,
            DriveMotor driveMotor,
            DriveEncoder driveEncoder,
            PIDController driveController,
            SimpleMotorFeedforward driveFeedforward) {
        m_experiments = experiments;
        m_driveMotor = driveMotor;
        m_driveEncoder = driveEncoder;
        m_driveController = driveController;
        m_driveFeedforward = driveFeedforward;
        SmartDashboard.putData(String.format("Swerve DriveServo %s", name), this);
    }

    void setDrive(SwerveModuleState state) {
        double speedM_S = state.speedMetersPerSecond;
          if (m_experiments.enabled(Experiment.UseClosedLoopDrive)) {
            offboard(speedM_S);
          } else {
            onboard(speedM_S);
         }

    }

    void set(double output) {
        m_driveMotor.set(output);
    }

    void offboard(double speedM_S) {
        m_driveMotor.setPID(ControlMode.Velocity, speedM_S);
    }

    void onboard(double speedM_S) {
        double accelM_S2 = (speedM_S - previousSpeedM_S) / 0.02; // TODO: measured dt
        previousSpeedM_S = speedM_S;
        driveMotorControllerOutput = m_driveController.calculate(getDriveSpeedMS(), speedM_S);
        driveFeedForwardOutput = m_driveFeedforward.calculate(speedM_S, accelM_S2);
        double driveOutput = driveMotorControllerOutput + driveFeedForwardOutput;
        // output deadband to prevent shivering.
        set(MathUtil.applyDeadband(driveOutput, m_config.kDriveDeadband));
    }

    double getDriveDistanceM() {
        return m_driveEncoder.getDistance();
    }

    double getDriveSpeedMS() {
        return m_driveEncoder.getRate();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Drive position (m)", () -> m_driveEncoder.getDistance(), null);
        builder.addDoubleProperty("Drive Speed (m/s)", () -> getDriveSpeedMS(), null);

        builder.addDoubleProperty("Drive Setpoint (m/s)", () -> m_driveController.getSetpoint(), null);
        builder.addDoubleProperty("Drive Speed Error (m/s)", () -> m_driveController.getPositionError(), null);
        builder.addDoubleProperty("Drive Accel Error (m/s/s)", () -> m_driveController.getVelocityError(), null);

        builder.addDoubleProperty("Controller Output", () -> driveMotorControllerOutput, null);
        builder.addDoubleProperty("Feed Forward Output", () -> driveFeedForwardOutput, null);

        builder.addDoubleProperty("Drive Motor Output [-1, 1]", () -> m_driveMotor.get(), null);

        builder.addDoubleProperty("m_driveSpeed", () -> m_driveSpeed, null);

    }

}
