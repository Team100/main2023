package org.team100.lib.subsystems.turning;

import edu.wpi.first.util.sendable.Sendable;

public interface TurningMotor extends Sendable {
    /** @return motor output in range [-1, 1] */
    double get();

    /** @param output motor output in range [-1, 1] */
    void set(double output);
}