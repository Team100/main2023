package org.team100.lib.motor.turning;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FalconTurningMotor implements TurningMotor, Sendable {
    public static class Config {
        public int kCurrentLimit = 40;
    }

    private final Config m_config = new Config();
    private final WPI_TalonFX m_motor;

    public FalconTurningMotor(String name, int canId) {
        m_motor = new WPI_TalonFX(canId);
        m_motor.configFactoryDefault();
        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.setInverted(InvertType.InvertMotorOutput);
        m_motor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, m_config.kCurrentLimit, m_config.kCurrentLimit, 0));
        m_motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, m_config.kCurrentLimit, m_config.kCurrentLimit, 0));
        SmartDashboard.putData(String.format("Falcon Turning Motor %s", name), this);
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
        builder.setSmartDashboardType("Falcon Turning Motor");
        builder.addDoubleProperty("Output", this::get, null);
    }
}
