package frc.robot;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.localization.TestAprilTagTest;

public class RobotPoseTest {
    

    // public robotPoseTest(){
        
    // }

    @Test
    public void testRobotPose(){
        System.out.println("YOOOO");

        Translation2d aprilTranslation2d = new Translation2d(0, 0);
        Rotation2d aprilRotation2d =  Rotation2d.fromDegrees(0);
        Pose2d aprilPose = new Pose2d(aprilTranslation2d, aprilRotation2d);

        TestAprilTagTest testAprilTag = new TestAprilTagTest(0, aprilPose);

        Translation2d tagTranslation2d = new Translation2d(1, 0);
        Rotation2d tagRotation2d =  Rotation2d.fromDegrees(45);
        Pose2d tagPose = new Pose2d(tagTranslation2d, tagRotation2d);

        Pose2d robotPose = toFieldCoordinates(tagTranslation2d, tagRotation2d, testAprilTag);

        // System.out.println(robotPose.getX());

        

        
        

    }

    private Pose2d toFieldCoordinates(Translation2d translation, Rotation2d rotation, TestAprilTagTest apriltag) {
        Transform2d robotRelative = new Transform2d(translation, rotation);
        Transform2d tagRelative = robotRelative.inverse();
        Pose2d fieldRelative = apriltag.getPose().plus(tagRelative);
        return fieldRelative;
    }


}
