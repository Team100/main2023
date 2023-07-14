package org.team100.lib.controller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class DriveControllers {
    public final PIDController xController;
    public final PIDController yController;
    // TODO i suspect there are duplicate controllers here
    public final ProfiledPIDController headingController;
    public final ProfiledPIDController thetaController;
    public final ProfiledPIDController rotateController;

    public DriveControllers(
            PIDController xController,
            PIDController yController,
            ProfiledPIDController headingController,
            ProfiledPIDController thetaController,
            ProfiledPIDController rotateController) {
        this.xController = xController;
        this.yController = yController;
        this.headingController = headingController;
        this.thetaController = thetaController;
        this.rotateController = rotateController;
    }
}
