// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class HashTag {
    // HashMap <Integer, TestAprilTag> aprilHash;
    TestAprilTag[] tags = new TestAprilTag[9];

    public HashTag(){
        // aprilHash = new HashMap<Integer, TestAprilTag>();
        // TestAprilTag tag1 = new TestAprilTag(2, poseCalc(0, 2, 0));
        // TestAprilTag tag2 = new TestAprilTag(8, poseCalc(1, 2, 0));
        // tags[1] = new TestAprilTag(1, poseCalc(4.19, -3.86, 0));
        tags[2] = new TestAprilTag(2, poseCalc(0, 0, 0));
        tags[3] = new TestAprilTag(3, poseCalc(0, 0, 0));


        // aprilHash.put(tag1.ID, tag1);
        // aprilHash.put(tag2.ID, tag2);
        // aprilHash.put(tag3.ID, tag3);
    }

    public TestAprilTag getCurrentTag(int i){
        // return aprilHash.get(i);
        // if(i == 3){
            // return new TestAprilTag(3, poseCalc(0, 0, 0));
        // }
        if(i >= tags.length){
            return null;
        } else {
            return tags[i];
        }
        
    }

    public Pose2d getTagIDPose(int i){
        TestAprilTag aprilTag = tags[i];
        return aprilTag.getPose();
    }
    
    public Pose2d poseCalc(double x, double y, double rads){
        Translation2d translation = new Translation2d(x, y);
        Rotation2d rotation = new Rotation2d(rads);
        Pose2d pose = new Pose2d(translation, rotation);
        return pose;
      }
}
