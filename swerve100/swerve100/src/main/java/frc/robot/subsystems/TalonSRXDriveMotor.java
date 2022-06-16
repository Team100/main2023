package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonSRXDriveMotor implements DriveMotor {
    private final WPI_TalonSRX m_motor;
    public static final double kDriveCurrentLimit = 10;

    public TalonSRXDriveMotor(String name, int canId) {
        m_motor = new WPI_TalonSRX(canId);
        m_motor.configFactoryDefault();
        m_motor.configSupplyCurrentLimit(
          new SupplyCurrentLimitConfiguration(true, kDriveCurrentLimit, 0, 0));
        // We're using the AndyMark CIMcoder, am-3314a.
        m_motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 0);
        m_motor.configFeedbackNotContinuous(false, 0); // just keep counting ticks
        m_motor.setSelectedSensorPosition(0); // TODO: is this required?
        SmartDashboard.putData(String.format("Talon SRX Drive Motor %s", name), this);
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

    public double getSelectedSensorVelocity() {
        return m_motor.getSelectedSensorVelocity();
    }

    public void configVelocityMeasurementWindow(int samplesToAverage) {
        m_motor.configVelocityMeasurementWindow(samplesToAverage);
    }

    public double configGetParameter(ParamEnum esamplevelocitywindow, int i) {
        return m_motor.configGetParameter(esamplevelocitywindow, i);
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    @Override
    public void set(double driveOutput) {
        m_motor.set(driveOutput);
    }

    public void setSensorPhase(boolean reverseDirection) {
        m_motor.setSensorPhase(reverseDirection);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("TalonSRXDriveMotor");
        builder.addDoubleProperty("Device ID", this::getDeviceID, null);
        builder.addDoubleProperty("Output", this::get, null);
    }
}
