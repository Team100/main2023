package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Minimal interface makes testing easier. */
public interface SwerveDriveSubsystemInterface extends Subsystem {

    Pose2d getPose();

    void stop();

    public void setDesiredState(SwerveState desiredState);

    @Deprecated
    void driveInFieldCoords(Twist2d twist2d);

    void truncate();
}
