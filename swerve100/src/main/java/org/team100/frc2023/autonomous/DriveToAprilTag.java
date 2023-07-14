package org.team100.frc2023.autonomous;

import java.util.function.Supplier;

import org.team100.frc2023.commands.GoalOffset;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveToAprilTag extends DriveToWaypoint2 {

    public DriveToAprilTag(
            Pose2d goal,
            double yOffset,
            Supplier<GoalOffset> goalSupplier,
            SwerveDriveSubsystem robotDrive,
            RedundantGyro gyro,
            Supplier<Double> gamePieceOffset) {
        super(goal, yOffset, goalSupplier, robotDrive, gyro, gamePieceOffset);
    }

    public static DriveToAprilTag newDriveToAprilTag(
            int tagID,
            double xOffset,
            double yOffset,
            Supplier<GoalOffset> goalSupplier,
            SwerveDriveSubsystem robotDrive,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            RedundantGyro gyro,
            Supplier<Double> gamePieceOffset) {
        Pose2d m_goal = goal(tagID, xOffset, layout);
        return new DriveToAprilTag(m_goal, yOffset, goalSupplier, robotDrive, gyro, gamePieceOffset);
    }

    public static Pose2d goal(int tagID, double xOffset, AprilTagFieldLayoutWithCorrectOrientation layout) {
        Transform2d m_offset = new Transform2d(new Translation2d(-xOffset, 0), new Rotation2d(0));
        Pose2d m_tagPose = layout.getTagPose(tagID).get().toPose2d();
        Pose2d m_goal = m_tagPose.plus(m_offset);
        return m_goal;
    }
}
