package org.team100.frc2023.commands;

import org.team100.lib.motion.drivetrain.SpeedLimits;

public enum DriveModerate {
    norm, slow, medium;

    public static SpeedLimits getSlow(){
        return new SpeedLimits(0.4, 1.0, 0.5, 1.0);
    }

    public static SpeedLimits getMedium(){
        return new SpeedLimits(4, 10, 2.5, 5);

    }
}

