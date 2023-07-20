package org.team100.lib.controller;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.drivetrain.SpeedLimits;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class DriveControllersFactory {
    public static class Config {
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
                return new DriveControllers(xController, yController, thetaController);
            case SWERVE_ONE:
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
                return new DriveControllers(xController, yController, thetaController);
            case SWERVE_TWO:
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
                return new DriveControllers(xController, yController, thetaController);

            default:
                // these RoboRIO's are have no drivetrains
                xController = new PIDController(1, 0.0, 0.0);
                yController = new PIDController(1, 0.0, 0.0);
                thetaController = new ProfiledPIDController(1, 0.0, 0.0, speedLimits.constraints());
                return new DriveControllers(xController, yController, thetaController);
        }
    }
}
