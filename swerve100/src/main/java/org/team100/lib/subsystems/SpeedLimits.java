package org.team100.lib.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SpeedLimits {
    public final double kMaxSpeedMetersPerSecond;
    public final double kMaxAccelerationMetersPerSecondSquared;
    public final double kMaxAngularSpeedRadiansPerSecond;
    public final double kMaxAngularSpeedRadiansPerSecondSquared;

    public SpeedLimits(
            double maxSpeedMetersPerSecond,
            double maxAccelerationMetersPerSecondSquared,
            double maxAngularSpeedRadiansPerSecond,
            double maxAngularSpeedRadiansPerSecondSquared) {
        kMaxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        kMaxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared;
        kMaxAngularSpeedRadiansPerSecond = maxAngularSpeedRadiansPerSecond;
        kMaxAngularSpeedRadiansPerSecondSquared = maxAngularSpeedRadiansPerSecondSquared;
    }

    public TrapezoidProfile.Constraints constraints() {
        return new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
