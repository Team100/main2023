package org.team100.lib.subsystems;

import org.team100.lib.sensors.RedundantGyro;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Corrects the tendency of the swerve drive to veer in the direction of
 * rotation, which is caused by delay in the sense->actuate loop.
 * 
 * Sources of delay include
 * 
 * * velocity window size
 * * velocity low-pass filtering
 * * steering controller delay
 * * robot period (actuation for 20 ms in the future, not just right now)
 * 
 * This issue is discussed in this CD thread:
 * https://www.chiefdelphi.com/t/field-relative-swervedrive-drift-even-with-simulated-perfect-modules/413892
 */
public class VeeringCorrection {
    /**
     * Delay in seconds.
     */
    private static final double kVeeringCorrection = 0.15;

    private final RedundantGyro m_gyro;

    public VeeringCorrection(RedundantGyro gyro) {
        m_gyro = gyro;
    }

    /**
     * Extrapolates the rotation based on the current angular velocity.
     * 
     * @param in past rotation
     * @return future rotation
     */
    public Rotation2d correct(Rotation2d in) {
        return in.minus(new Rotation2d(m_gyro.getRedundantGyroRate() * kVeeringCorrection));

    }

}
