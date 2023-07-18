package org.team100.lib.subsystems.drive;

public interface DriveMotor {
    /** @return Drive motor output in range [-1, 1] */
    double get();

    /** @param output Drive motor output in range [-1, 1] */
    void set(double output);
}