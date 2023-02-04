// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
@SuppressWarnings("unused")
public class HashTag {
    private HashMap <Integer, TestAprilTag> aprilHash;

    public HashTag(){
        aprilHash = new HashMap<Integer, TestAprilTag>();

        TestAprilTag tag1 = new TestAprilTag(3, new Pose3d ( new Translation3d(0, 0 ,0), new Rotation3d()) );
        // TestAprilTag tag2 = new TestAprilTag(2, new Pose3d(new Translation3d(15.513558, 0, 2.748026), new Rotation3d()));
        // TestAprilTag tag3 = new TestAprilTag(1, new Pose3d(new Translation3d(15.513558, 0, 4.424426), new Rotation3d()));

        aprilHash.put(tag1.ID, tag1);
        //aprilHash.put(tag2.ID, tag2);
    }

    public TestAprilTag getTag(int i){
        if (aprilHash.containsKey(i)) {
            return aprilHash.get(i);
        }
        return null;
    }

    public Pose3d getTagIDPose(int i){
        TestAprilTag aprilTag = getTag(i);
        if (aprilTag == null) {
            return null;
        }
        return aprilTag.getPose();
    }
    
}
