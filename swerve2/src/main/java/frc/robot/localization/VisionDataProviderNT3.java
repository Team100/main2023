// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class VisionDataProviderNT3 {
    NetworkTable table;

    double[] xValues, yValues, zValues, idValues;
    String[] xRot, yRot, zRot;
    double[] defaultValue = new double[0];
    String[] strDefaultValue = new String[0];
    
    HashMap <Integer, ReferenceAprilTag> aprilHash;
    HashTag hashtag;

    /**
     * The amount of time it takes for the vision system to process an image and send the results over NetworkTables.
     */
    private double kProcessingDelay = 0.3;

    public VisionDataProviderNT3(){
        table = NetworkTableInstance.getDefault().getTable("Vision");
        hashtag = new HashTag();
    }

    /**
     * Updates the values of the apriltags from NetworkTables.
     * This method should be called periodically, for instance from the robot's or a subsystem's periodic method.
     */
    public void periodic() {
        idValues = table.getEntry("id").getDoubleArray(defaultValue);
        xValues = table.getEntry("pose_t_x").getDoubleArray(defaultValue);
        yValues = table.getEntry("pose_t_y").getDoubleArray(defaultValue);
        zValues = table.getEntry("pose_t_z").getDoubleArray(defaultValue);
        xRot = table.getEntry("pose_R_x").getStringArray(strDefaultValue);
        xRot = table.getEntry("pose_R_y").getStringArray(strDefaultValue);
        xRot = table.getEntry("pose_R_z").getStringArray(strDefaultValue);
    }

    /**
     * Retrieves all apriltags from NetworkTables and computes the current estimated pose for each.
     * Attaches the current timestamp to each of the poses, and returns them all in an array.
     * @return Array of estimated poses (one for each april tag). May be empty if no tags are detected.
     */
    public VisionEstimate[] getPoseEstimates(){
        double timestamp = Timer.getFPGATimestamp();
        VisionEstimate[] estimates = new VisionEstimate[idValues.length];
        for(int i = 0; i < idValues.length; i++){
            // Create a translation and rotation from the current apriltag
            Translation2d translation = new Translation2d(xValues[i], zValues[i]);
            Rotation2d rotation = new Rotation2d(getRot(i));

            // Convert the apriltag pose to a field-relative pose, and add it to the array
            Pose2d estimatedPose = toFieldCoordinates(translation, rotation, hashtag.getCurrentTag((int)idValues[i]));
            estimates[i] = new VisionEstimate(estimatedPose, timestamp - kProcessingDelay);
        }
        return estimates;
    }

    /**
     * Gets a Rotation2d representing the rotation of the apriltag given, using data from NetworkTables.
     * @param i Index of apriltag in NetworkTables data
     * @return Rotation2d representing the rotation of the apriltag
     */
    public double getRot(int i){
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