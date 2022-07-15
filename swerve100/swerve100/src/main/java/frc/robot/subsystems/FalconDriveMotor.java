package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;

public class FalconDriveMotor implements DriveMotor {
    private final WPI_TalonFX m_motor;
    public static final double kDriveCurrentLimit = 10;

    public FalconDriveMotor(int canId) {
        m_motor = new WPI_TalonFX(canId);
        m_motor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kDriveCurrentLimit, kDriveCurrentLimit, 0));
        m_motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, kDriveCurrentLimit, kDriveCurrentLimit, 0));
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    @Override
    public void set(double output) {
        m_motor.set(output);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("TalonFXDriveMotor");
        builder.addDoubleProperty("Device ID", () -> m_motor.getDeviceID(), null);
        builder.addDoubleProperty("Output", this::get, null);
    }
}