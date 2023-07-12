package org.team100.frc2023.commands;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveMobility extends CommandBase {
    private static final double kCommunitySizeMeters = 5.8;
    private static final double kXSpeed1_1 = 0.3;

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
        if (m_robotDrive.getPose().getX() < kCommunitySizeMeters) {
            m_robotDrive.drive(kXSpeed1_1, 0, 0, true);
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
        m_robotDrive.drive(0, 0, 0, false);
    }
}
