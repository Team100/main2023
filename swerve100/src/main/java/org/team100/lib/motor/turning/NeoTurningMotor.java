package org.team100.lib.motor.turning;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NeoTurningMotor implements TurningMotor, Sendable {
    public static class Config {
        public int kCurrentLimit = 40;
    }

    private SparkMaxPIDController m_pidController;
    private final Config m_config = new Config();
    private final CANSparkMax m_motor;

    public NeoTurningMotor(String name, int canId) {
        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        m_motor.setInverted(true);
        m_motor.setSmartCurrentLimit(m_config.kCurrentLimit);
        m_pidController = m_motor.getPIDController();
        SmartDashboard.putData(String.format("Neo Turning Motor %s", name), this);
        m_pidController.setPositionPIDWrappingEnabled(true);
        m_pidController.setP(.1);
        m_pidController.setI(0);
        m_pidController.setD(0);
        m_pidController.setIZone(0);
        m_pidController.setFF(0);
        m_pidController.setOutputRange(-1, 1);
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    @Override
    public void set(double output) {
        m_motor.set(output);
    }

    public void setPIDVelocity(double output, double Accel) {
        double motorGearing = 1;//I believe there are not any robotics that actually use a neo turning motor I was changing this class to be more ready in case we do
        final ControlType controlType = CANSparkMax.ControlType.kVelocity;
        m_pidController.setReference(motorGearing * output, controlType);
    }

    // THIS DOES NOT ACTUALLY SET PID This is just here for the other turning motors
    public void setPIDPosition(double output) {
        this.set(output);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("NeoTurningMotor");
        builder.addDoubleProperty("Device ID", () -> m_motor.getDeviceId(), null);
        builder.addDoubleProperty("Output", this::get, null);
    }

}
