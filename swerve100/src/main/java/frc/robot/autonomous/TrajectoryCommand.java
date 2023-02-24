
package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public abstract class TrajectoryCommand extends SwerveControllerCommand {
    SwerveDriveSubsystem m_robotDrive;

    public TrajectoryCommand(SwerveDriveSubsystem m_robotDrive, Trajectory trajectory) {

        super(
                trajectory,
                m_robotDrive::getPose,
                SwerveDriveSubsystem.kDriveKinematics,
                m_robotDrive.xController,
                m_robotDrive.yController,
                m_robotDrive.thetaController,
                () -> new Rotation2d(),
                m_robotDrive::setModuleStates,
                m_robotDrive);
        this.m_robotDrive = m_robotDrive;
        addRequirements(this.m_robotDrive);
    }
}
