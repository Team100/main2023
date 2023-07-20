package org.team100.lib.controller;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.drivetrain.SpeedLimits;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class DriveControllersFactory {
    public static class Config {
        public PidGains compBotHeadingGain = new PidGains(0.67, 0, 0);
        public PidGains swerveOneHeadingGain = new PidGains(0.5, 0, 0);
        public PidGains swerveTwoHeadingGain = new PidGains(1.0, 0, 0.15);

        public double compBotHeadingIntegratorRange = 0.1;
        public double swerveOneHeadingIntegratorRange = 0.1;
        public double swerveTwoHeadingIntegratorRange = 0.5;

        public double compBotHeadingTolerance = 0.01;
        public double swerveOneHeadingTolerance = 0.002;
        public double swerveTwoHeadingTolerance = 0.002;

        public PidGains thetaGain = new PidGains(3.0, 0, 0);

        public PidGains compBotCartesianGain = new PidGains(0.15, 0, 0);
        public PidGains swerveOneCartesianGain = new PidGains(0.15, 0, 0);
        public PidGains swerveTwoCartesianGain = new PidGains(2.0, 0.1, 0.15);

        public double swerveTwoCartesianIntegratorRange = 0.5;

        public double compBotCartesianTolerance = 0.01;
        public double swerveOneCartesianTolerance = 0.2;
        public double swerveTwoCartesianTolerance = 0.01;
    }

    private final Config m_config = new Config();

    public DriveControllers get(Identity identity, SpeedLimits speedLimits) {
        switch (identity) {
            case COMP_BOT:
                var headingController = new ProfiledPIDController(
                        m_config.compBotHeadingGain.p,
                        m_config.compBotHeadingGain.i,
                        m_config.compBotHeadingGain.d,
                        speedLimits.constraints());
                headingController.setIntegratorRange(
                        m_config.compBotHeadingIntegratorRange * -1.0,
                        m_config.compBotHeadingIntegratorRange);
                headingController.setTolerance(m_config.compBotHeadingTolerance);
                
                var xController = new PIDController(
                        m_config.compBotCartesianGain.p,
                        m_config.compBotCartesianGain.i,
                        m_config.compBotCartesianGain.d);
                xController.setTolerance(m_config.compBotCartesianTolerance);

                var yController = new PIDController(
                        m_config.compBotCartesianGain.p,
                        m_config.compBotCartesianGain.i,
                        m_config.compBotCartesianGain.d);
                yController.setTolerance(m_config.compBotCartesianTolerance);

                var thetaController = new ProfiledPIDController(
                        m_config.thetaGain.p,
                        m_config.thetaGain.i,
                        m_config.thetaGain.d,
                        speedLimits.constraints());
                return new DriveControllers(xController, yController, headingController, thetaController);
            case SWERVE_ONE:
                headingController = new ProfiledPIDController(
                        m_config.swerveOneHeadingGain.p,
                        m_config.swerveOneHeadingGain.i,
                        m_config.swerveOneHeadingGain.d,
                        speedLimits.constraints());
                headingController.setIntegratorRange(
                        m_config.swerveOneHeadingIntegratorRange * -1.0,
                        m_config.swerveOneHeadingIntegratorRange);
                headingController.setTolerance(m_config.swerveOneHeadingTolerance);

                xController = new PIDController(
                        m_config.swerveOneCartesianGain.p,
                        m_config.swerveOneCartesianGain.i,
                        m_config.swerveOneCartesianGain.d);
                xController.setTolerance(m_config.swerveOneCartesianTolerance);

                yController = new PIDController(
                        m_config.swerveOneCartesianGain.p,
                        m_config.swerveOneCartesianGain.i,
                        m_config.swerveOneCartesianGain.d);
                yController.setTolerance(m_config.swerveOneCartesianTolerance);

                thetaController = new ProfiledPIDController(
                        m_config.thetaGain.p,
                        m_config.thetaGain.i,
                        m_config.thetaGain.d,
                        speedLimits.constraints());
                return new DriveControllers(xController, yController, headingController, thetaController);
            case SWERVE_TWO:
                headingController = new ProfiledPIDController(
                        m_config.swerveTwoHeadingGain.p,
                        m_config.swerveTwoHeadingGain.i,
                        m_config.swerveTwoHeadingGain.d,
                        speedLimits.constraints());
                headingController.setIntegratorRange(
                        m_config.swerveTwoHeadingIntegratorRange * -1.0,
                        m_config.swerveTwoHeadingIntegratorRange);
                headingController.setTolerance(m_config.swerveTwoHeadingTolerance);

                xController = new PIDController(
                        m_config.swerveTwoCartesianGain.p,
                        m_config.swerveTwoCartesianGain.i,
                        m_config.swerveTwoCartesianGain.d);
                xController.setTolerance(m_config.swerveTwoCartesianTolerance);
                xController.setIntegratorRange(
                        m_config.swerveTwoCartesianIntegratorRange * -1.0,
                        m_config.swerveTwoCartesianIntegratorRange);

                yController = new PIDController(
                        m_config.swerveTwoCartesianGain.p,
                        m_config.swerveTwoCartesianGain.i,
                        m_config.swerveTwoCartesianGain.d);
                yController.setTolerance(m_config.swerveTwoCartesianTolerance);
                yController.setIntegratorRange(
                        m_config.swerveTwoCartesianIntegratorRange * -1.0,
                        m_config.swerveTwoCartesianIntegratorRange);

                thetaController = new ProfiledPIDController(
                        m_config.thetaGain.p,
                        m_config.thetaGain.i,
                        m_config.thetaGain.d,
                        speedLimits.constraints());
                return new DriveControllers(xController, yController, headingController, thetaController);

            default:
                // these RoboRIO's are have no drivetrains
                headingController = new ProfiledPIDController(1, 0, 0, speedLimits.constraints());
                xController = new PIDController(1, 0.0, 0.0);
                yController = new PIDController(1, 0.0, 0.0);
                thetaController = new ProfiledPIDController(1, 0.0, 0.0, speedLimits.constraints());
                return new DriveControllers(xController, yController, headingController, thetaController);
        }
    }
}
