
package frc.robot.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.commands.SwerveControllerCommand;
import frc.robot.subsystems.AHRSClass;
import frc.robot.subsystems.SwerveDriveSubsystem;

public abstract class TrajectoryCommand extends SwerveControllerCommand {
    SwerveDriveSubsystem m_robotDrive;

    public TrajectoryCommand(SwerveDriveSubsystem m_robotDrive, Trajectory trajectory, AHRSClass gyro) {

        super(
                trajectory,
                m_robotDrive::getPose,
                SwerveDriveSubsystem.kDriveKinematics,
                m_robotDrive.xController,
                m_robotDrive.yController,
                m_robotDrive.thetaController,
                () -> new Rotation2d(),
                m_robotDrive::setModuleStates,
                gyro,
                m_robotDrive);
        this.m_robotDrive = m_robotDrive;
        addRequirements(this.m_robotDrive);
    }
    public TrajectoryCommand(SwerveDriveSubsystem m_robotDrive, Trajectory trajectory, Supplier<Rotation2d> desiredRotation, AHRSClass gyro) {

        super(
                trajectory,
                m_robotDrive::getPose,
                SwerveDriveSubsystem.kDriveKinematics,
                m_robotDrive.xController,
                m_robotDrive.yController,
                m_robotDrive.thetaController,
                desiredRotation,
                m_robotDrive::setModuleStates,
                gyro,
                m_robotDrive);
        this.m_robotDrive = m_robotDrive;
        addRequirements(this.m_robotDrive);
    }
}
