package org.team100.lib.motor.turning;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PWMTurningMotor implements TurningMotor, Sendable {
    private final PWMMotorController m_motor;

    public PWMTurningMotor(String name, int channel) {
        m_motor = new VictorSP(channel);
        m_motor.setInverted(true);
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

    // THIS DOES NOT ACTUALLY SET PID This is just here for the other turning motors
    public void setPIDVelocity(double output, double Accel) {
        this.set(output);
    }

    public void setPIDPosition(double output) {
        this.set(output);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PWMTurningMotor");
        builder.addDoubleProperty("Device ID", () -> m_motor.getChannel(), null);
        builder.addDoubleProperty("Output", this::get, null);
    }

}
