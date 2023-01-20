package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ContinuousServoTurningMotor implements TurningMotor {
    private final Parallax360 m_motor;

    public ContinuousServoTurningMotor(String name, int channel) {
        m_motor = new Parallax360(name, channel);
        SmartDashboard.putData(String.format("Continous Servo Turning Motor %s", name), this);
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    @Override
    public void set(double output) {
        m_motor.set(output);
    }

    public int getChannel() {
        return m_motor.getChannel();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ContinuousServoTurningMotor");
        builder.addDoubleProperty("Channel", this::getChannel, null);
        builder.addDoubleProperty("Output", this::get, null);
    }
    
}
