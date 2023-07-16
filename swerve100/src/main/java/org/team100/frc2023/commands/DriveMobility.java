package org.team100.frc2023.commands;

import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveMobility extends CommandBase {
    public static class Config {
        public double kCommunitySizeMeters = 5.8;
        public double kXSpeedM_S = 1.5;
    }

    private final Config m_config = new Config();
    private final SwerveDriveSubsystem m_robotDrive;

    private boolean done;

    /** Drive forward, exiting the community area. */
    public DriveMobility(SwerveDriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        if (m_robotDrive.getPose().getX() < m_config.kCommunitySizeMeters) {
            m_robotDrive.driveMetersPerSec(new Twist2d(m_config.kXSpeedM_S, 0, 0), true);
        } else {
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.driveMetersPerSec(new Twist2d(0, 0, 0), false);
    }
}
