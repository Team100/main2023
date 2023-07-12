package org.team100.frc2023.autonomous;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToThreshold extends CommandBase {
    private final SwerveDriveSubsystem m_robotDrive;
    private boolean done;

    public DriveToThreshold(SwerveDriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        // TODO: replace this with a waypoint
        if (m_robotDrive.getPose().getX() > 4.1) {
            m_robotDrive.drive(-0.4, 0, 0, true);
        } else {
            m_robotDrive.drive(0, 0, 0, true);
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
