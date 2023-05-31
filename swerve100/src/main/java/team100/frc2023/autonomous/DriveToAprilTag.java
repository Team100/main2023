package team100.frc2023.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import team100.frc2023.commands.GoalOffset;
import team100.frc2023.localization.AprilTagFieldLayoutWithCorrectOrientation;
import team100.frc2023.subsystems.AHRSClass;
import team100.frc2023.subsystems.SwerveDriveSubsystem;

public class DriveToAprilTag extends DriveToWaypoint2 {

    public DriveToAprilTag(
            Pose2d goal,
            double yOffset,
            Supplier<GoalOffset> goalSupplier,
            SwerveDriveSubsystem robotDrive,
            AHRSClass gyro,
            Supplier<Double> gamePieceOffset) {
        super(goal, yOffset, goalSupplier, robotDrive, gyro, gamePieceOffset);
    }

    public static DriveToAprilTag newDriveToAprilTag(
            int tagID,
            double xOffset,
            double yOffset,
            Supplier<GoalOffset> goalSupplier,
            SwerveDriveSubsystem robotDrive,
            AHRSClass gyro,
            Supplier<Double> gamePieceOffset) {
        Pose2d m_goal = goal(tagID, xOffset, robotDrive.visionDataProvider.layout);
        return new DriveToAprilTag(m_goal, yOffset, goalSupplier, robotDrive, gyro, gamePieceOffset);
    }

    public static Pose2d goal(int tagID, double xOffset, AprilTagFieldLayoutWithCorrectOrientation layout) {
        Transform2d m_offset = new Transform2d(new Translation2d(-xOffset, 0), new Rotation2d(0));
        Pose2d m_tagPose = layout.getTagPose(tagID).get().toPose2d();
        // System.out.println(m_tagPose);
        Pose2d m_goal = m_tagPose.plus(m_offset);
        return m_goal;
    }
}
