package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FalconDriveMotor implements DriveMotor {
    private final WPI_TalonFX m_motor;

    public FalconDriveMotor(String name, int canId, double kDriveCurrentLimit) {
        m_motor = new WPI_TalonFX(canId);
        m_motor.configFactoryDefault();

        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kDriveCurrentLimit, kDriveCurrentLimit, 0));
        m_motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, kDriveCurrentLimit, kDriveCurrentLimit, 0));
        SmartDashboard.putData(String.format("Falcon Drive Motor %s", name), this);
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    @Override
    public void set(double output) {
        m_motor.set(MathUtil.clamp(output, -1.0, 1.0));
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
