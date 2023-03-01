package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FalconDriveMotor implements DriveMotor {
    private final WPI_TalonFX m_motor;

    public FalconDriveMotor(String name, int canId, double kDriveCurrentLimit) {
        m_motor = new WPI_TalonFX(canId);
        m_motor.configFactoryDefault();
        m_motor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kDriveCurrentLimit, kDriveCurrentLimit, 0));
        m_motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, kDriveCurrentLimit, kDriveCurrentLimit, 0));
        m_motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_20Ms, 0);
        m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 20);
        SmartDashboard.putData(String.format("Falcon Drive Motor %s", name), this);
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    public void robotInit() {
        m_motor.config_kF(0, 0, 0);
		m_motor.config_kP(0, 1, 0);
		m_motor.config_kI(0, 1, 0);
		m_motor.config_kD(0, 1, 0);
    }

    @Override
    // public void set(double output) {
    public void set(double outputMs) {
        // m_motor.set(output);
        double wheelCircumferenceMeters = 0.1016 * Math.PI;
        double outputTurnsPerSec = outputMs / wheelCircumferenceMeters;
        double outputTicksPer100Ms = outputTurnsPerSec * 204.8;
        m_motor.set(ControlMode.Velocity, outputTicksPer100Ms);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("FalconDriveMotor");
        builder.addDoubleProperty("Device ID", () -> m_motor.getDeviceID(), null);
        builder.addDoubleProperty("Output", this::get, null);
    }

    public TalonFXSensorCollection getSensorCollection() {
        return m_motor.getSensorCollection();
    }
}
