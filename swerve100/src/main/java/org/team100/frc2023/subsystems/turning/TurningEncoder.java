package org.team100.frc2023.subsystems.turning;

import edu.wpi.first.util.sendable.Sendable;

public interface TurningEncoder extends Sendable {
    /**
     * @return Module azimuth angle in radians, counterclockwise-positive.
     *         Accumulates multiple turns; if you need the modulus, use
     *         MathUtil.AngleModulus.
     */
    double getAngle();

    /**
     * Resets angle to zero.
     */
    void reset();
}