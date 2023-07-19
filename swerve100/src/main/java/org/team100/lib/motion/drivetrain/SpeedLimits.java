package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SpeedLimits {
    public final double kMaxSpeedMetersPerSecond;
    public final double kMaxAccelerationMetersPerSecondSquared;
    public final double kMaxJerkMetersPerSecondCubed;
    public final double kMaxAngularSpeedRadiansPerSecond;
    public final double kMaxAngularAccelRad_SS;
    public final double kMaxAngularJerkRadiansPerSecondCubed;

    public SpeedLimits(
            double maxSpeedMetersPerSecond,
            double maxAccelerationMetersPerSecondSquared,
            double maxAngularSpeedRadiansPerSecond,
            double maxAngularSpeedRadiansPerSecondSquared) {
        kMaxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        kMaxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared;
        kMaxJerkMetersPerSecondCubed = 0; // actually means infinite
        kMaxAngularSpeedRadiansPerSecond = maxAngularSpeedRadiansPerSecond;
        kMaxAngularAccelRad_SS = maxAngularSpeedRadiansPerSecondSquared;
        kMaxAngularJerkRadiansPerSecondCubed = 0; // actually means infinite
    }

    public TrapezoidProfile.Constraints constraints() {
        return new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelRad_SS);
    }
}
