package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FalconDriveEncoder implements DriveEncoder {
    private static final int kIntegratedSensorPulsesPerTurn = 2048;
    private final FalconDriveMotor m_motor;
    private final double m_distancePerPulse;
    private final boolean m_reverse;

    public FalconDriveEncoder(String name,
            FalconDriveMotor motor,
            double distancePerTurn,
            boolean reverseDirection) {
        this.m_motor = motor;
        this.m_reverse = reverseDirection;
        this.m_distancePerPulse = distancePerTurn / kIntegratedSensorPulsesPerTurn;
        m_motor.setInverted(reverseDirection);
        SmartDashboard.putData(String.format("Falcon Drive Encoder %s", name), this);
    }

    @Override
    public double getDistance() {
        return m_motor.getSensorCollection().getIntegratedSensorPosition() * m_distancePerPulse;
    }

    @Override
    public double getRate() {
        // sensor velocity is 1/2048ths of a turn per 100ms
        return m_motor.getSensorCollection().getIntegratedSensorVelocity()
                * 10 * m_distancePerPulse;
    }

    @Override
    public void reset() {
        m_motor.getSensorCollection().setIntegratedSensorPosition(0, 0);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("FalconDriveEncoder");
        builder.addDoubleProperty("Speed", this::getRate, null);
    }

}
