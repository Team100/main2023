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
    HashMap <Integer, ReferenceAprilTag> aprilHash;

    public HashTag(){
        aprilHash = new HashMap<Integer, ReferenceAprilTag>();
        ReferenceAprilTag tag1 = new ReferenceAprilTag(0, poseCalc(0, 1, 0));
        ReferenceAprilTag tag2 = new ReferenceAprilTag(1, poseCalc(1, 1, 0));


        aprilHash.put(tag1.ID, tag1);
        aprilHash.put(tag2.ID, tag2);
    }

    public ReferenceAprilTag getCurrentTag(int i){
        return aprilHash.get(i);
        
    }
    
    public Pose2d poseCalc(double x, double z, double rads){
        Translation2d translation = new Translation2d(x, z);
        Rotation2d rotation = new Rotation2d(rads);
        Pose2d pose = new Pose2d(translation, rotation);
        return pose;
      }
}
