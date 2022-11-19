package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DutyCycleDriveEncoder implements DriveEncoder {
    private final DutyCycleEncoder m_encoder;
    private final LinearFilter m_filter;
    private double m_previousValue;
    private long m_previousTimeNs;

    public DutyCycleDriveEncoder(
        String name,
        int channel,
        double gearRatio,
        boolean reversed) {
        m_encoder = new DutyCycleEncoder(channel);
        m_encoder.setDistancePerRotation((reversed ? -1 : 1) * 2 * Math.PI / gearRatio);
        m_encoder.setDutyCycleRange(0.027, 0.971); // parallax 360 PWM
        m_filter = LinearFilter.singlePoleIIR(10, 1);
        SmartDashboard.putData(String.format("Duty Cycle Drive Encoder %s", name), this);
    }

    @Override
    public double getRate() {
        double newValue = m_encoder.get();
        double deltaValue = newValue - m_previousValue;
        long newTimeNs = RobotController.getFPGATime();
        double deltaTimeNs = newTimeNs - m_previousTimeNs;
        double rateTurnsPerSec = 1e6 * deltaValue / deltaTimeNs;
        m_previousValue = newValue;
        m_previousTimeNs = newTimeNs;
        return m_filter.calculate(rateTurnsPerSec);
    }

    @Override
    public void reset() {
        m_encoder.reset();
    }

    public int getChannel() {
        return m_encoder.getSourceChannel();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("DutyCycleDriveEncoder");
        builder.addDoubleProperty("Channel", this::getChannel, null);
        builder.addDoubleProperty("Rate", this::getRate, null);
    }
}
