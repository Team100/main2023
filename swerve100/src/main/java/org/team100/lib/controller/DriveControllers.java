package org.team100.lib.controller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class DriveControllers {
    public final PIDController xController;
    public final PIDController yController;
    public final ProfiledPIDController thetaController;

    public DriveControllers(
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController
            ) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
    }
}
