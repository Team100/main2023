package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;

/** Minimal interface makes testing easier. */
public interface SwerveDriveSubsystemInterface {

    Pose2d getPose();

    void stop();

    void setDesiredState(SwerveState desiredState);

    void truncate();

    /** Because Subsystem is now concrete, it needs an accessor. */
    SwerveDriveSubsystem get();
}
