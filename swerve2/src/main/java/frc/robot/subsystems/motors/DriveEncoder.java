package frc.robot.subsystems.motors;

import edu.wpi.first.util.sendable.Sendable;

public interface DriveEncoder extends Sendable {
    double getRate();
    double getDistance();
    void reset();
}