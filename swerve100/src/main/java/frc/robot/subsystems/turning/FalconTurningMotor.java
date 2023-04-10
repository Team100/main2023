package frc.robot.subsystems.turning;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FalconTurningMotor implements TurningMotor {
    private final WPI_TalonFX m_motor;
    public static final int kTurningCurrentLimit = 40;

    /**
     * Throws if any of the configurations fail.
     */
    public FalconTurningMotor(String name, int canId) {
        m_motor = new WPI_TalonFX(canId);
        m_motor.configFactoryDefault();
        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.setInverted(InvertType.InvertMotorOutput);
        m_motor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kTurningCurrentLimit, kTurningCurrentLimit, 0));
        m_motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, kTurningCurrentLimit, kTurningCurrentLimit, 0));
        SmartDashboard.putData(String.format("Falcon Turning Motor %s", name), this);
    }

    private void require(ErrorCode errorCode) {
        if (errorCode != ErrorCode.OK)
            throw new IllegalArgumentException("motor configuration error: " + errorCode.name());
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
        // builder.addDoubleProperty("Device ID", () -> m_motor.getDeviceId(), null);
        builder.addDoubleProperty("Output", this::get, null);
    }
}
