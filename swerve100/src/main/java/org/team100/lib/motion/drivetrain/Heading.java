package org.team100.lib.motion.drivetrain;

import org.team100.lib.sensors.RedundantGyro;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * To make sure we calculate heading the same way everywhere. This is NWU,
 * counterclockwise-positive.
 */
public class Heading implements HeadingInterface {
    private final RedundantGyro m_gyro;

    public Heading(RedundantGyro gyro) {
        m_gyro = gyro;
    }

    @Override
    public Rotation2d getHeadingNWU() {
        double yawNED = m_gyro.getRedundantYawNED();
        // invert NED to get NWU
        return Rotation2d.fromDegrees(-1.0 * yawNED);
    }

    @Override
    public double getHeadingRateNWU() {
        double rateNED = m_gyro.getRedundantGyroRateNED();
        // invert NED to get NWU
        return -1.0 * rateNED;
    }

}