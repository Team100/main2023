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
    
        tags[2] = new TestAprilTag(2, new Pose2d(0, 0, new Rotation2d(0)));
       
    }

    public TestAprilTag getCurrentTag(int i){
        if(tags[i] == null){
            return null;
        }else{
            return tags[i];
        }
    }

    public Pose2d getTagIDPose(int i){
        TestAprilTag aprilTag = tags[i];
        return aprilTag.getPose();
    }
    

