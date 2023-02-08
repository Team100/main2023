package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AnalogTurningEncoder implements TurningEncoder {
    private final AnalogEncoder m_encoder;

    /**
     * @param channel
     * @param inputOffset unit = turns, i.e. [0,1]
     * @param gearRatio
     */
    public AnalogTurningEncoder(
            String name,
            int channel,
            double inputOffset,
            double gearRatio) {
        m_encoder = new AnalogEncoder(channel);
        m_encoder.setPositionOffset(inputOffset);
        m_encoder.setDistancePerRotation(2.0 * Math.PI / gearRatio);

        SmartDashboard.putData(String.format("Analog Turning Encoder %s", name), this);
    }

    /**
     * Counterclockwise-positive
     */
    @Override
    public double getAngle() {
        return m_encoder.getDistance();
    }

    @Override
    public void reset() {
        m_encoder.reset();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("AnalogTurningEncoder");
        builder.addDoubleProperty("Channel", () -> m_encoder.getChannel(), null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("Turns", () -> m_encoder.get(), null);
        builder.addDoubleProperty("absolute", () -> m_encoder.getAbsolutePosition(), null);
    }
}
