package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;

public interface TurningEncoder extends Sendable {
    /**
     * 
     * @return module azimuth angle in radians
     */
    double getAngle();
    void reset();
}