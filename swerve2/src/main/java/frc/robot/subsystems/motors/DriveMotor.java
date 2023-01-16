package frc.robot.subsystems.motors;

import edu.wpi.first.util.sendable.Sendable;

public interface DriveMotor extends Sendable {
    double get();
    void set(double output);
}