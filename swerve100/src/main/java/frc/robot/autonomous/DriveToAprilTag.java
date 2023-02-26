package frc.robot.autonomous;

import frc.robot.localization.AprilFieldLayout2;
import frc.robot.localization.VisionDataProvider;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.ResourceBundle.Control;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveToAprilTag extends DriveToWaypoint2 {

    static Transform2d m_offset = new Transform2d(new Translation2d(1, 0), new Rotation2d(Math.PI));
    private static Control m_control;
    
    public DriveToAprilTag(Pose2d goalSupplier, SwerveDriveSubsystem robotDrive) {
        super(goalSupplier, robotDrive);
    }

    public static DriveToAprilTag newDriveToAprilTag(int tagID, SwerveDriveSubsystem robotDrive) {
        Pose2d m_goal = goal(tagID, robotDrive.visionDataProvider.layout);
        return new DriveToAprilTag(m_goal, robotDrive);
    }

    public static Pose2d goal(int tagID, AprilTagFieldLayout layout) {
        Pose2d m_tagPose = layout.getTagPose(tagID).get().toPose2d();
        System.out.println(m_tagPose);
        Pose2d m_goal = m_tagPose.plus(m_offset);
        return m_goal;
    }
}
