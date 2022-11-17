package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DutyCycleTurningEncoder implements TurningEncoder {
    private final DutyCycleEncoder m_encoder;

    public DutyCycleTurningEncoder(
            String name,
            int channel,
            double offset,
            double gearRatio,
            boolean reversed) {
        m_encoder = new DutyCycleEncoder(channel);
        m_encoder.setPositionOffset(offset);
        m_encoder.setDistancePerRotation((reversed ? -1 : 1) * 2 * Math.PI / gearRatio);
        m_encoder.setDutyCycleRange(0.027, 0.971); // parallax 360 PWM
        SmartDashboard.putData(String.format("Duty Cycle Turning Encoder %s", name), this);
    }

    @Override
    public double getAngle() {
        return m_encoder.getDistance();
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
        builder.setSmartDashboardType("DutyCycleTurningEncoder");
        builder.addDoubleProperty("Channel", this::getChannel, null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
    }
}
