package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;

public class PoseTest {
    public static Pose2d toFieldCoordinates(Translation2d translation2d, Rotation2d rotation2d, AprilTag apriltag) {
        Transform2d robotRelative = new Transform2d(translation2d, rotation2d);
        Transform2d tagRelative = robotRelative.inverse();
        Pose2d fieldRelative = apriltag.getPose().plus(tagRelative);
        return fieldRelative;
    }
}
