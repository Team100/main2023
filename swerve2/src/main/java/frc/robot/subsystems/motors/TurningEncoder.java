package frc.robot.subsystems.motors;

import edu.wpi.first.util.sendable.Sendable;

public interface TurningEncoder extends Sendable {
    /**
     * 
     * @return module azimuth angle in radians
     */
    double getAngle();
    void reset();
}