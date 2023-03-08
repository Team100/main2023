package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.autoLevel;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class VasiliAutonomous extends SequentialCommandGroup {
    public VasiliAutonomous(SwerveDriveSubsystem m_robotDrive) {
        addCommands(
                // TODO add place cone/cube command here
                moveFromStartingPoseToGamePiece.newMoveFromStartingPoseToGamePiece(
                        m_robotDrive,
                        () -> m_robotDrive.getPose(),
                        new Pose2d(5.7, 0.92, new Rotation2d())));
    }
}
