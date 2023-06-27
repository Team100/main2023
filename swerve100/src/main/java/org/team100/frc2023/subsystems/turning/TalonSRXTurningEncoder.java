package org.team100.frc2023.subsystems.turning;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonSRXTurningEncoder implements TurningEncoder{
    private final WPI_TalonSRX m_motor;

    public TalonSRXTurningEncoder(String name, CANTurningMotor motor) {
        m_motor = motor.getMotor();
        SmartDashboard.putData(String.format("TalonSRXEncoder %s", name), this);
    }

    @Override
    public double getAngle() {
        return m_motor.getSelectedSensorPosition()/1666*2*Math.PI;
    }

    @Override
    public void reset() {
        m_motor.setSelectedSensorPosition(0);
        return;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("TalonSRXTurningEncoder");
        builder.addDoubleProperty("Angle", () -> this.getAngle(), null);
    }
}