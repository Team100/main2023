// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class VisionDataProviderNT4 {
    DoubleArraySubscriber idSub, xSub, ySub, zSub;
    StringArraySubscriber xRotSub, yRotSub, zRotSub;    
    HashMap <Integer, ReferenceAprilTag> aprilHash;
    HashTag hashtag;

    /**
     * The amount of time it takes for the vision system to process an image and send the results over NetworkTables.
     */
    private double kProcessingDelay = 0.3;

    public VisionDataProviderNT4(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");
        idSub = table.getDoubleArrayTopic("id").subscribe(new double[0]);
        xSub = table.getDoubleArrayTopic("pose_t_x").subscribe(new double[0]);
        ySub = table.getDoubleArrayTopic("pose_t_y").subscribe(new double[0]);
        zSub = table.getDoubleArrayTopic("pose_t_z").subscribe(new double[0]);
        xRotSub = table.getStringArrayTopic("pose_R_x").subscribe(new String[0]);
        yRotSub = table.getStringArrayTopic("pose_R_y").subscribe(new String[0]);
        zRotSub = table.getStringArrayTopic("pose_R_z").subscribe(new String[0]);

        hashtag = new HashTag();
    }

    /**
     * Retrieves all apriltags from NetworkTables and computes the current estimated pose for each.
     * Attaches the current timestamp to each of the poses, and returns them all in an array.
     * @return Array of estimated poses (one for each april tag). May be empty if no tags are detected.
     */
    public VisionEstimate[] getPoseEstimates(){
        // Fetch the data from NetworkTables
        double[] idValues = idSub.get();
        double[] xValues = xSub.get();
        double[] zValues = zSub.get();
        String[] xRotValues = xRotSub.get();
        String[] zRotValues = zRotSub.get();

        double timestamp = Timer.getFPGATimestamp();
        VisionEstimate[] estimates = new VisionEstimate[idValues.length];
        
        // Loop through all the apriltags detected and compute the estimated robot pose for each
        for(int i = 0; i < idValues.length; i++){
            // Create a translation and rotation from the current apriltag
            Translation2d translation = new Translation2d(xValues[i], zValues[i]);
            Rotation2d rotation = new Rotation2d(getRot(i, xRotValues, zRotValues));

            // Convert the apriltag pose to a field-relative pose, and add it to the array
            Pose2d estimatedPose = toFieldCoordinates(translation, rotation, hashtag.getCurrentTag((int)idValues[i]));
            estimates[i] = new VisionEstimate(estimatedPose, timestamp - kProcessingDelay);
        }
        return estimates;
    }

    /**
     * Gets a Rotation2d representing the rotation of the apriltag given, using data from NetworkTables.
     * @param i Index of apriltag in NetworkTables data
     * @param xRot Array of x-rotations from NetworkTables
     * @param zRot Array of z-rotations from NetworkTables
     * @return Rotation2d representing the rotation of the apriltag
     */
    public double getRot(int i, String[] xRot, String[] zRot){
        String a = stringtoVal(xRot[i]);
        String g = stringtoVal(zRot[i]);
        Rotation2d rotation = new Rotation2d(
                Double.parseDouble(a), 
                Double.parseDouble(g));
        return rotation.getRadians();
    }

    /** 
     * Takes an input string and returns the first part of the string before the first space.
     * <p>For example, "1.0 2.0 3.0" would return "1.0".</p>
     * <p>A string with no spaces would be returned in its entirety.</p>
     * @param str Input string
     * @return Part of the input string up to (but not including) the first space
     */
    public String stringtoVal(String str){
        // Switch to new method later since using NetworkTables3
        String out = "";
        for(int i = 0; i<str.length(); i++){
            if(str.charAt(i) != ' '){
                out = out + str.charAt(i);
            } else {
                break;
            }
        }
        return out;
    }

    /**
     * Converts a robot-relative apriltag pose to a field-relative robot pose, given the current april tag.
     * @param translation2d Location of the april tag relative to the robot, usually retrieved from a vision system
     * @param rotation2d Rotation of the april tag relative to the robot, usually retrieved from a vision system
     * @param apriltag Known pose of the april tag, in field coordinates
     * @return Field-relative robot pose
     */
    public static Pose2d toFieldCoordinates(Translation2d translation2d, Rotation2d rotation2d, ReferenceAprilTag apriltag) {
        Transform2d robotRelative = new Transform2d(translation2d, rotation2d);
        Transform2d tagRelative = robotRelative.inverse();
        Pose2d fieldRelative = apriltag.getPose().plus(tagRelative);
        return fieldRelative;
    }
}