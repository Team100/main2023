package frc.robot.autonomous;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToAprilTag extends CommandBase{

    Pose2d m_tagPose;
    SwerveDriveSubsystem m_robotDrive;
    Pose2d m_goal;
    Transform2d m_offset = new Transform2d(new Translation2d(-1, 0), new Rotation2d());

    public DriveToAprilTag(SwerveDriveSubsystem robotDrive, int tagID) {
        m_robotDrive = robotDrive;
        m_tagPose = m_robotDrive.visionDataProvider.layout.getTagPose(tagID).get().toPose2d();
        m_goal = m_tagPose.plus(m_offset);
    }
    @Override
    public void initialize() {
        new DriveToWaypoint2(m_goal, m_robotDrive);
    }
}
