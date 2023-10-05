package org.team100.lib.motion.drivetrain;

import org.team100.lib.config.Identity;

public class SpeedLimitsFactory {
    /**
     * @param showMode is for younger drivers to drive the robot slowly.
     */
    public static SpeedLimits get(Identity identity, boolean showMode) {
        switch (identity) {
            case COMP_BOT:
                if (showMode)
                    return new SpeedLimits(1, 2, 1, 1);
                return new SpeedLimits(5, 10, 5, 5);
            case SWERVE_TWO:
                return new SpeedLimits(5, 10, 5, 5);
            case SWERVE_ONE:
                return new SpeedLimits(5, 10, 5, 5);
            case BLANK:
                return new SpeedLimits(5, 10, 5, 5);
            case CAMERA_DOLLY:
                return new SpeedLimits(5, 10, 5, 5);
            default:
                return new SpeedLimits(1, 1, 1, 1);
            // previously this would throw
            // throw new IllegalStateException("Identity is not swerve: " +
            // Identity.get().name());
        }
    }

    public static SpeedLimits getSpeedLimits(SpeedLimit limit){

        switch(limit){
            case Fast:
                return new SpeedLimits(5, 10, 5, 5);
            case Medium:
                return new SpeedLimits(2, 10, 0.4, 5);
            case Slow:
                return new SpeedLimits(0.4, 1.0, 0.5, 1.0);
            default:
                return new SpeedLimits(0, 0, 0, 0);
        }
            

    }
}
