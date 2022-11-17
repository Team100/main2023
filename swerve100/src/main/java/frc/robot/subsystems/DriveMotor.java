package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;

public interface DriveMotor extends Sendable {
    double get();
    void set(double output);
}