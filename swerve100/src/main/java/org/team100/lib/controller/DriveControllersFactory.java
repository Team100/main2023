package org.team100.lib.controller;

import org.team100.frc2023.LQRManager;
import org.team100.lib.config.Identity;
import org.team100.lib.motion.drivetrain.SpeedLimits;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DriveControllersFactory {
    public static class Config {
        public PidGains thetaGain = new PidGains(3.0, 0, 0, 0.01, 0.01, true);
        LinearSystem<N2, N1, N1> m_translationPlant = LinearSystemId.identifyPositionSystem(1.3, 0.06);
        public LQRGains compBotCartesianGain = new LQRGains(5, 5, m_translationPlant, .015, .17, .01, .05, 1,20);
        public LQRGains swerveOneCartesianGain = new LQRGains(5, 5, m_translationPlant, .015, .17, .01, .05, 1,20);
        // public PidGains swerveOneCartesianGain = new PidGains(2, 0.1, 0.15, 0.1, 0.01, false);
        public LQRGains swerveTwoCartesianGain = new LQRGains(5, 5, m_translationPlant, .015, .17, .01, .05, 1,20);
    }

    private final Config m_config = new Config();

    public DriveControllers get(Identity identity, SpeedLimits speedLimits) {
        switch (identity) {
            case COMP_BOT:
                return new DriveControllers(
                    LQR(m_config.compBotCartesianGain),
                    LQR(m_config.compBotCartesianGain),
                    pid(m_config.thetaGain));
            case SWERVE_ONE:
                return new DriveControllers(
                    LQR(m_config.swerveOneCartesianGain),
                    LQR(m_config.swerveOneCartesianGain),
                    pid(m_config.thetaGain));
            case SWERVE_TWO:
                return new DriveControllers(
                    LQR(m_config.swerveTwoCartesianGain),
                    LQR(m_config.swerveTwoCartesianGain),
                    pid(m_config.thetaGain));
            default:
                // these RoboRIO's are have no drivetrains
                LinearSystem<N2, N1, N1> m_translationPlant = LinearSystemId.identifyPositionSystem(1.3, 0.06);
                return new DriveControllers(
                        new LQRManager(5, 5, m_translationPlant, .015, .17, .01, .05, 1,20),
                        new LQRManager(5, 5, m_translationPlant, .015, .17, .01, .05, 1,20),
                        new PIDController(1, 0.0, 0.0));
        }
    }

    private static PIDController pid(PidGains g) {
        
        PIDController pid = new PIDController(g.p, g.i, g.d);
        pid.setIntegratorRange(-1.0 * g.integratorRange, g.integratorRange);
        pid.setTolerance(g.tolerance);
        if (g.continuous)
            pid.enableContinuousInput(-1.0 * Math.PI, Math.PI);
        return pid;
    }
    private static LQRManager LQR(LQRGains g) {
        LQRManager manager = new LQRManager(g);
        return manager;
    }
}
