package frc.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Uses default position/velocity sensor which is the integrated one.
 * 
 * Configures the velocity reporting period to 20ms (default 100) and sampling
 * to 16 (default 64).
 * 
 * See details on velocity averaging and sampling.
 * https://v5.docs.ctr-electronics.com/en/stable/ch14_MCSensor.html#velocity-measurement-filter
 */
public class FalconDriveMotor implements DriveMotor {
    private final WPI_TalonFX m_motor;

    /**
     * Throws if any of the configurations fail.
     */
    public FalconDriveMotor(String name, int canId, double kDriveCurrentLimit) {
        m_motor = new WPI_TalonFX(canId);
        require(m_motor.configFactoryDefault());
        m_motor.setNeutralMode(NeutralMode.Brake);
        require(m_motor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kDriveCurrentLimit, kDriveCurrentLimit, 0)));
        require(m_motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, kDriveCurrentLimit, kDriveCurrentLimit, 0)));

        // default is 100 ms, i.e. lots of smoothing. robot loop is 20 ms, so this seems
        // like a good maximum.
        require(m_motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_20Ms));

        // default is 64 samples i.e. 64 ms worth of samples. try 16 to kinda match the
        // period?
        require(m_motor.configVelocityMeasurementWindow(16));

        SmartDashboard.putData(String.format("Falcon Drive Motor %s", name), this);
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
        m_motor.setVoltage(12 * MathUtil.clamp(output, -1.0, 1.0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("FalconDriveMotor");
        builder.addDoubleProperty("Device ID", () -> m_motor.getDeviceID(), null);
        builder.addDoubleProperty("Output", this::get, null);
    }

    /**
     * @return integrated sensor position in sensor units (1/2048 turn).
     */
    double getPosition() {
        return m_motor.getSelectedSensorPosition();
    }

    /**
     * @return integrated sensor velocity in sensor units (1/2048 turn) per 100ms.
     */
    double getVelocity() {
        return m_motor.getSelectedSensorVelocity();
    }

    /**
     * Sets integrated sensor position to zero.
     * Throws if it fails, watch out, don't use this during a match.
     */
    void resetPosition() {
        require(m_motor.setSelectedSensorPosition(0));
    }
}
