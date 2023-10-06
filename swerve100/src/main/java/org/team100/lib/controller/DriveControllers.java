package org.team100.lib.controller;

import org.team100.frc2023.LQRManager;

import edu.wpi.first.math.controller.PIDController;

public class DriveControllers {
    public final LQRManager xController;
    public final LQRManager yController;
    public final PIDController thetaController;

    public DriveControllers(
        LQRManager xController,
        LQRManager yController,
            PIDController thetaController
            ) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
    }
}
