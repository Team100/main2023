package org.team100.frc2023.subsystems.turning;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.util.sendable.Sendable;

public interface TurningMotor extends Sendable {
    /** @return motor output in range [-1, 1] */
    double get();

    /** @param output motor output in range [-1, 1] */
    void set(double output);

    void setPID(ControlMode control, double output);
}