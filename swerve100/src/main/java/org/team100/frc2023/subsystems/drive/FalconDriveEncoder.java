package org.team100.frc2023.subsystems.drive;

import org.team100.lib.subsystems.drive.DriveEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FalconDriveEncoder implements DriveEncoder {
    private static final int kIntegratedSensorPulsesPerTurn = 2048;
    private final FalconDriveMotor m_motor;
    private final double m_distancePerPulse;

    /** @param distancePerTurn in meters */
    public FalconDriveEncoder(String name,
            FalconDriveMotor motor,
            double distancePerTurn) {
        this.m_motor = motor;
        this.m_distancePerPulse = distancePerTurn / kIntegratedSensorPulsesPerTurn;
        SmartDashboard.putData(String.format("Falcon Drive Encoder %s", name), this);
    }

    @Override
    public double getDistance() {
        return m_motor.getPosition() * m_distancePerPulse;
    }

    @Override
    public double getRate() {
        // sensor velocity is 1/2048ths of a turn per 100ms
        return m_motor.getVelocity() * 10 * m_distancePerPulse;
    }

    @Override
    public void reset() {
        m_motor.resetPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("FalconDriveEncoder");
        builder.addDoubleProperty("Speed Ms", this::getRate, null);
    }

}
