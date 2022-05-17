package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AnalogDriveEncoder implements DriveEncoder {
    private final AnalogEncoder m_encoder;
    
    private double m_previousValueTurns;
    private long m_previousTimeNs;

    public AnalogDriveEncoder(String name, int channel) {
        m_encoder = new AnalogEncoder(channel);
        SmartDashboard.putData(String.format("Analog Drive Encoder %s", name), this);
    }

    // TODO: this method works poorly at low speeds, add a buffer.
    @Override
    public double getRate() {
        double newValueTurns = m_encoder.get();
        double deltaValueTurns = newValueTurns - m_previousValueTurns;
        long newTimeNs = RobotController.getFPGATime();
        double deltaTimeNs = newTimeNs - m_previousTimeNs;
        double rateTurnsPerSec = 1e6 * deltaValueTurns / deltaTimeNs;
        m_previousValueTurns = newValueTurns;
        m_previousTimeNs = newTimeNs;
        return rateTurnsPerSec;
    }

    @Override
    public void reset() {
        m_encoder.reset();        
    }

    public int getChannel() {
        return m_encoder.getChannel();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("AnalogDriveEncoder");
        builder.addDoubleProperty("Channel", this::getChannel, null);
    }
}
