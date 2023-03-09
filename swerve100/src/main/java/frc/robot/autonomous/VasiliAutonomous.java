package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class VasiliAutonomous extends SequentialCommandGroup {
    /** Creates a new autonomous. */
    public VasiliAutonomous(SwerveDriveSubsystem m_robotDrive) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        Rotation2d desiredRots = new Rotation2d(Math.PI);
        SwerveModuleState[] desiredStates = new SwerveModuleState[]{
            new SwerveModuleState(0, desiredRots),
            new SwerveModuleState(0, desiredRots), 
            new SwerveModuleState(0, desiredRots), 
            new SwerveModuleState(0, desiredRots)
        };
        CommandBase command = new CommandBase() {
            @Override
            public void initialize() {
                m_robotDrive.setModuleStates(desiredStates);
            };
            @Override
            public boolean isFinished() {
                return false;
            }
        };
        command.addRequirements(m_robotDrive);
        addCommands(
                // TODO add place cone/cube command here
                moveFromStartingPoseToGamePiece
                .newMoveFromStartingPoseToGamePiece(
                        m_robotDrive,
                        new Pose2d(
                                m_robotDrive.getPose().getX(),
                                m_robotDrive.getPose().getY(),
                                new Rotation2d(Math.PI / 2)),
                        new Pose2d(0, 0.01, new Rotation2d())),
                moveFromStartingPoseToGamePiece
                        .newMoveFromStartingPoseToGamePiece(
                                m_robotDrive,
                                new Pose2d(
                                        m_robotDrive.getPose().getX(),
                                        m_robotDrive.getPose().getY(),
                                        new Rotation2d(Math.PI / 2)),
                                new Pose2d(0, 0.92, new Rotation2d())),
                moveFromStartingPoseToGamePiece
                        .newMoveFromStartingPoseToGamePiece(
                                m_robotDrive,
                                new Pose2d(
                                        m_robotDrive.getPose().getX(),
                                        m_robotDrive.getPose().getY(),
                                        new Rotation2d()),
                                new Pose2d(0.01, 0, new Rotation2d())),
                moveFromStartingPoseToGamePiece
                        .newMoveFromStartingPoseToGamePiece(
                                m_robotDrive,
                                new Pose2d(
                                        m_robotDrive.getPose().getX(),
                                        m_robotDrive.getPose().getY(),
                                        new Rotation2d()),
                                new Pose2d(5.7, 0, new Rotation2d())));

    }
}
