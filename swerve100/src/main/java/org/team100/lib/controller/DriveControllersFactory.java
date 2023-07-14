package org.team100.lib.controller;

import org.team100.lib.config.Identity;
import org.team100.lib.subsystems.SpeedLimits;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;

public class DriveControllersFactory {
    public static DriveControllers get(Identity identity, SpeedLimits speedLimits) {
        switch (identity) {
            case COMP_BOT:
                var headingController = new ProfiledPIDController(0.67, 0, 0, speedLimits.constraints());
                headingController.setIntegratorRange(-0.1, 0.1);
                // Note very low heading tolerance.
                headingController.setTolerance(0.01);

                var rotateController = new ProfiledPIDController(1, 0.5, 0, speedLimits.constraints());
                rotateController.setIntegratorRange(-0.2, 0.2);

                var xController = new PIDController(0.15, 0.0, 0.0);
                xController.setTolerance(0.01);

                var yController = new PIDController(0.15, 0.0, 0.0);
                yController.setTolerance(0.01);

                var thetaController = new ProfiledPIDController(3.0, 0.0, 0.0, speedLimits.constraints());

                return new DriveControllers(xController, yController, headingController, thetaController,
                        rotateController);
            case SWERVE_TWO:
                headingController = new ProfiledPIDController(1, 0, 0.15, speedLimits.constraints());
                headingController.setIntegratorRange(-0.5, 0.5);
                // Note very low heading tolerance.
                headingController.setTolerance(Units.degreesToRadians(0.1));

                rotateController = new ProfiledPIDController(0.7, 0, 0, speedLimits.constraints());

                xController = new PIDController(2, 0.1, 0.0);
                xController.setTolerance(0.01);
                xController.setIntegratorRange(-0.5, 0.5);

                yController = new PIDController(2, 0.1, 0.0);
                yController.setTolerance(0.01);
                yController.setIntegratorRange(-0.5, 0.5);

                thetaController = new ProfiledPIDController(3, 0.0, 0.0, speedLimits.constraints());
                return new DriveControllers(xController, yController, headingController, thetaController,
                        rotateController);
            case SWERVE_ONE:
                headingController = new ProfiledPIDController(0.5, 0, 0, speedLimits.constraints());
                headingController.setIntegratorRange(-0.1, 0.1);
                // Note very low heading tolerance.
                headingController.setTolerance(Units.degreesToRadians(0.1));

                rotateController = new ProfiledPIDController(0.7, 0, 0, speedLimits.constraints());

                xController = new PIDController(0.15, 0.0, 0.0);
                xController.setTolerance(0.2);

                yController = new PIDController(0.15, 0.0, 0.0);
                yController.setTolerance(0.2);

                thetaController = new ProfiledPIDController(3.0, 0.0, 0.0, speedLimits.constraints());
                return new DriveControllers(xController, yController, headingController, thetaController,
                        rotateController);
            case BLANK:
                headingController = new ProfiledPIDController(1, .5, 0.15, speedLimits.constraints());
                headingController.setIntegratorRange(-0.1, 0.1);
                // Note very low heading tolerance.
                headingController.setTolerance(Units.degreesToRadians(0.1));

                rotateController = new ProfiledPIDController(0.7, 0, 0, speedLimits.constraints());

                xController = new PIDController(0.15, 0.0, 0.0);
                xController.setTolerance(0.2);

                yController = new PIDController(0.15, 0.0, 0.0);
                yController.setTolerance(0.2);

                thetaController = new ProfiledPIDController(3.0, 0.0, 0.0, speedLimits.constraints());
                return new DriveControllers(xController, yController, headingController, thetaController,
                        rotateController);

            case CAMERA_DOLLY:
                headingController = new ProfiledPIDController(1, 0, 0,
                        speedLimits.constraints());
                headingController.setIntegratorRange(-0.1, 0.1);
                // Note very low heading tolerance.
                headingController.setTolerance(Units.degreesToRadians(0.1));

                rotateController = new ProfiledPIDController(0.7, 0, 0,
                        speedLimits.constraints());

                xController = new PIDController(0.15, 0.0, 0.0);
                xController.setTolerance(0.01);

                yController = new PIDController(0.15, 0.0, 0.0);
                yController.setTolerance(0.01);

                thetaController = new ProfiledPIDController(3.0, 0.0, 0.0,
                        speedLimits.constraints());
                return new DriveControllers(xController, yController, headingController, thetaController,
                        rotateController);
            default:
                throw new IllegalStateException("Identity is not swerve: " + Identity.get().name());

        }
    }

}
