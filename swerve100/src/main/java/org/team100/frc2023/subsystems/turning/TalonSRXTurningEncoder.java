package org.team100.frc2023.subsystems.turning;

import org.team100.lib.encoder.turning.TurningEncoder;
import org.team100.lib.motor.turning.CANTurningMotor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonSRXTurningEncoder implements TurningEncoder, Sendable{
    private final WPI_TalonSRX m_motor;

    public TalonSRXTurningEncoder(String name, CANTurningMotor motor) {
        m_motor = motor.getMotor();
        SmartDashboard.putData(String.format("TalonSRXEncoder %s", name), this);
    }

    @Override
    public double getAngle() {
        return MathUtil.angleModulus(m_motor.getSelectedSensorPosition()/1666*2*Math.PI);
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

    @Override
    public void close() {
    }
}