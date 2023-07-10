package org.team100.frc2023.subsystems.turning;

import org.team100.lib.subsystems.turning.TurningMotor;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CANTurningMotor implements TurningMotor {
    private final WPI_VictorSPX m_motor;
    private final int channel;
    public static final double kTurningCurrentLimit = 10;

    public CANTurningMotor(String name, int channel) {
        m_motor = new WPI_VictorSPX(channel);
        this.channel = channel;
        SmartDashboard.putData(String.format("CAN Turning Motor %s", name), this);
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
        builder.addDoubleProperty("Device ID", () -> channel, null);
        builder.addDoubleProperty("Output", this::get, null);
    }

}
