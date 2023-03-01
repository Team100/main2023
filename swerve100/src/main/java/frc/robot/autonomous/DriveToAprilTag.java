package frc.robot.autonomous;

import frc.robot.commands.GoalOffset;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.function.Supplier;

import frc.robot.localization.AprilTagFieldLayoutWithCorrectOrientation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveToAprilTag extends DriveToWaypoint2 {

    // TODO: Change depending on task
    static Transform2d m_offset = new Transform2d(new Translation2d(-1, 0), new Rotation2d(0));

    public DriveToAprilTag(Pose2d goal, Supplier<GoalOffset> goalSupplier, SwerveDriveSubsystem robotDrive) {
        super(goal, goalSupplier, robotDrive);
    }

    public static DriveToAprilTag newDriveToAprilTag(int tagID, Supplier<GoalOffset> goalSupplier,
            SwerveDriveSubsystem robotDrive) {
        Pose2d m_goal = goal(tagID, robotDrive.visionDataProvider.layout);
        return new DriveToAprilTag(m_goal, goalSupplier, robotDrive);
    }

    public static Pose2d goal(int tagID, AprilTagFieldLayoutWithCorrectOrientation layout) {
        Pose2d m_tagPose = layout.getTagPose(tagID).get().toPose2d();
        System.out.println(m_tagPose);
        Pose2d m_goal = m_tagPose.plus(m_offset);
        return m_goal;
    }
}
