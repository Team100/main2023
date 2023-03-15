package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;

public interface DriveMotor extends Sendable {
    /** @return Drive motor output in range [-1, 1] */
    double get();

    /** @param output Drive motor output in range [-1, 1] */
    void set(double output);
}