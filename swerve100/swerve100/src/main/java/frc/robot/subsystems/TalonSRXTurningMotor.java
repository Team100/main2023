package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonSRXTurningMotor implements TurningMotor {
    private final WPI_TalonSRX m_motor;
    public static final double kTurningCurrentLimit = 5;
    
    public TalonSRXTurningMotor(String name, int canId) {
        m_motor = new WPI_TalonSRX(canId);
        m_motor.configFactoryDefault();
        m_motor.configSupplyCurrentLimit(
          new SupplyCurrentLimitConfiguration(true, kTurningCurrentLimit, 0, 0));
        // We're using the AndyMark MA3, am-2899, on the steering shaft.
        m_motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0);
        m_motor.configFeedbackNotContinuous(true, 0); // just return the angle
        SmartDashboard.putData(String.format("Talon SRX Turning Motor %s", name), this);
    }

	public int getDeviceID() {
		return m_motor.getDeviceID();
	}

    public double getSelectedSensorPosition() {
        return m_motor.getSelectedSensorPosition();
    }

    public SensorCollection getSensorCollection() {
        return m_motor.getSensorCollection();
    }

    public void setSelectedSensorPosition(int i) {
        m_motor.setSelectedSensorPosition(i);
    }

    public void setSensorPhase(boolean reverseDirection) {
        m_motor.setSensorPhase(reverseDirection);
    }

    public double configGetParameter(ParamEnum efeedbacksensortype, int i) {
        return m_motor.configGetParameter(efeedbacksensortype, i);
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    @Override
    public void set(double turnOutput) {
        m_motor.set(turnOutput);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("TalonSRXTurningMotor");
        builder.addDoubleProperty("Device ID", this::getDeviceID, null);
        builder.addDoubleProperty("Output", this::get, null);
    }
}
