package org.team100.frc2023.autonomous;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToThreshold extends CommandBase {
    private static final double kEdgeOfRampMeters = 4.1;
    private static final double kXSpeedM_S = -2.0;

    private final SwerveDriveSubsystem m_robotDrive;
    
    private boolean done;

    /** Drive back to the edge of the charge station. */
    public DriveToThreshold(SwerveDriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        if (m_robotDrive.getPose().getX() > kEdgeOfRampMeters) {
            m_robotDrive.driveMetersPerSec(new Twist2d(kXSpeedM_S, 0, 0), true);
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
