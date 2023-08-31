package org.team100.lib.motor.turning;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
        m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.setInverted(InvertType.InvertMotorOutput);
        m_motor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, m_config.kCurrentLimit, m_config.kCurrentLimit, 0));
        m_motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, m_config.kCurrentLimit, m_config.kCurrentLimit, 0));
        SmartDashboard.putData(String.format("Falcon Turning Motor %s", name), this);
        m_motor.configNominalOutputForward(0);
        m_motor.configNominalOutputReverse(0);
        m_motor.config_kF(0, 0);
        m_motor.config_kP(0, 0.05);
        m_motor.config_kI(0, 0);
        m_motor.config_kD(0, 0);
        m_motor.configVoltageCompSaturation(11);
        m_motor.enableVoltageCompensation(true);
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    public void setPIDVelocity(double outputRadiansPerSec, double outputRadiansPerSecPerSec) {
        double gearRatio = 10.29;
        double ticksPerRevolution = 2048;
        double revolutionsPerSec = outputRadiansPerSec/(2*Math.PI);
        double revolutionsPerSec2 = 0;
        double revsPer100ms = revolutionsPerSec / 10;
        double ticksPer100ms = revsPer100ms * ticksPerRevolution;
        DemandType type = DemandType.ArbitraryFeedForward;
        double Kn = 0.75/6.6*gearRatio;//1.296
        double Kf = 0.6;//0.3145
        double Ke = 0.0688420763;
        double Ks = 0.05/6.6*gearRatio;
        double VSat = 11;
        if (revolutionsPerSec<.25) {
            Ks = .3/6.6*gearRatio;
          }
        double kFF = (Kn*revolutionsPerSec + Ks*Math.signum(revolutionsPerSec))/VSat;
        m_motor.set(ControlMode.Velocity, ticksPer100ms*gearRatio, type, kFF);
        }

    public void setPIDPosition(double outputRadians) {
        double gearRatio = 10.29;
        double ticksPerRevolution = 2048;
        double outputTicks = outputRadians / (Math.PI * 2) * ticksPerRevolution;
        m_motor.set(ControlMode.Position, outputTicks*gearRatio);
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
