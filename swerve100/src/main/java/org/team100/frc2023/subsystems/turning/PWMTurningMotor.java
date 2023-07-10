package org.team100.frc2023.subsystems.turning;

import org.team100.lib.subsystems.turning.TurningMotor;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PWMTurningMotor implements TurningMotor {
    private final PWMMotorController m_motor;
    public static final double kTurningCurrentLimit = 10;

    public PWMTurningMotor(String name, int channel) {
        m_motor = new VictorSP(channel);
        SmartDashboard.putData(String.format("PWM Turning Motor %s", name), this);
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
        builder.setSmartDashboardType("PWMTurningMotor");
        builder.addDoubleProperty("Device ID", () -> m_motor.getChannel(), null);
        builder.addDoubleProperty("Output", this::get, null);
    }
    
}
