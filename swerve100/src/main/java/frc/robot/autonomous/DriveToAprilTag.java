package frc.robot.autonomous;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveToAprilTag extends DriveToWaypoint2 {

    static Transform2d m_offset = new Transform2d(new Translation2d(-1, 0), new Rotation2d());

    public DriveToAprilTag(Pose2d goal, SwerveDriveSubsystem robotDrive) {
        super(goal, robotDrive);
    }

    public static DriveToAprilTag newDriveToAprilTag(int tagID, SwerveDriveSubsystem robotDrive) {
        Pose2d m_tagPose = robotDrive.visionDataProvider.layout.getTagPose(tagID).get().toPose2d();
        return new DriveToAprilTag(m_tagPose.plus(m_offset), robotDrive);
    }
}
