package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;

public interface TurningEncoder extends Sendable {
    double getAngle();
    void reset();
}