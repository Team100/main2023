package org.team100.lib.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

/** To make sure we calculate heading the same way everywhere. */
public class Heading {
    private final RedundantGyro m_gyro;

    public Heading(RedundantGyro gyro) {
        m_gyro = gyro;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-m_gyro.getRedundantYaw());
    }

}
