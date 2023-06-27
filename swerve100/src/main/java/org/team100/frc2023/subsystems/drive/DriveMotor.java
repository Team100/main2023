package org.team100.frc2023.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.util.sendable.Sendable;

public interface DriveMotor extends Sendable {
    /** @return Drive motor output in range [-1, 1] */
    double get();

    /** @param output Drive motor output in range [-1, 1] */
    void set(double output);

    void setPID(ControlMode control, double output);
}