package org.team100.lib.subsystems.drive;

import edu.wpi.first.util.sendable.Sendable;

public interface DriveEncoder extends Sendable {
    /** @return encoder rate in meters per second */
    double getRate();

    /** @return accumulated distance in meters */
    double getDistance();

    /** set distance to zero */
    void reset();
}