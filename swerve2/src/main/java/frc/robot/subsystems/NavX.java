package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

// NavX wrapper
public class NavX implements Sendable{
    private AHRS m_gyro;
    private boolean m_gyroReversed;

    public NavX(AHRS gyro, boolean gyroReversed) {
        m_gyro = gyro;
        m_gyroReversed = gyroReversed;
    }

    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    // Returns Rotation2d, accounting for gyro reversal
    public Rotation2d getRotation2d() {
        return m_gyro.getRotation2d();
    }

    public double getRate() {
        return -m_gyro.getRate() * (m_gyroReversed ? -1.0 : 1.0);
    }

    public void reset() {
        m_gyro.reset();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("NavX");
        builder.addDoubleProperty("Heading", this::getHeading, null);
        builder.addDoubleProperty("Rate", this::getRate, null);
    }
}