package org.team100.lib.controller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class DriveControllers {
    public final PIDController xController;
    public final PIDController yController;
    // TODO i suspect there are duplicate controllers here
    public final ProfiledPIDController headingController; // only used by DriveWithHeading.
    public final ProfiledPIDController thetaController; // used by several commands
    public final PIDController rotateController; // only used by Rotate.

    public DriveControllers(
            PIDController xController,
            PIDController yController,
            ProfiledPIDController headingController,
            ProfiledPIDController thetaController,
            PIDController rotateController
            ) {
        this.xController = xController;
        this.yController = yController;
        this.headingController = headingController;
        this.thetaController = thetaController;
        this.rotateController = rotateController;
    }
}
