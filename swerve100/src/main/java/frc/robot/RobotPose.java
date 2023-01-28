// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class RobotPose {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");
    double [] xValues;
    double [] yValues;
    double [] zValues;
    double[] defaultValue = new double[0];
    String[] strDefaultValue = new String[0];
    double [] idValues;
    String [] xRot;
    String [] yRot;
    String [] zRot;
    String a;
    String g;
    String h;
    String i;
    HashMap <Integer, TestAprilTag> aprilHash;
    HashTag hashtag;

    public RobotPose(){
        table = NetworkTableInstance.getDefault().getTable("Vision");
        hashtag = new HashTag();
    }

    public boolean aprilPresent(){
        // idValues = table.getEntry("id").getDoubleArray(defaultValue);
        // if(idValues.length > 0){
        //     return true;
        // }
        return false;
    }
    
    public int getID(int i){
        idValues = table.getEntry("id").getDoubleArray(defaultValue);
        return (int)idValues[i];
    }
    public int getIDlength(){
        idValues = table.getEntry("id").getDoubleArray(defaultValue);
        return idValues.length;
    }
    public double getPosX(int i){
        xValues = table.getEntry("pose_t_x").getDoubleArray(defaultValue);
        return xValues[i];
    }

    public double getPosY(int i){
        yValues = table.getEntry("pose_t_y").getDoubleArray(defaultValue);
        return yValues[i];
    }

    public double getPosZ(int i){
        zValues = table.getEntry("pose_t_z").getDoubleArray(defaultValue);
        return zValues[i];
    }

    public String getRotX(int i){
        xRot = table.getEntry("pose_R_x").getStringArray(strDefaultValue);
        return xRot[i];
    }

    public String getRotY(int i){
        yRot = table.getEntry("pose_R_y").getStringArray(strDefaultValue);
        return yRot[i];
    }

    public String getRotZ(int i){
        zRot = table.getEntry("pose_R_z").getStringArray(strDefaultValue);
        return zRot[i];
    }

    public double getRot(int i){
        a = stringtoVal(getRotX(i));
        g = stringtoVal(getRotZ(i));
        Rotation2d rotation = new Rotation2d(Double.parseDouble(a), Double.parseDouble(g));
        SmartDashboard.putNumber("RotationPoseForRobot", rotation.getRadians());
        return rotation.getRadians();
    }

    public double getPitchRot(){
        h = stringtoVal(getRotZ(0));
        i = stringtoVal(getRotZ(0));
        Rotation2d rotation = new Rotation2d(Double.parseDouble(h), Double.parseDouble(i));
        SmartDashboard.putNumber("Rotation Pitch For Robot", rotation.getRadians());
        return rotation.getDegrees();

    }

    public Pose2d getRobotPose(int i){
        Translation2d translation = new Translation2d(getPosX(i), getPosZ(i));
        Rotation2d rotation = new Rotation2d(getRot(i));
        Pose2d robotPose = toFieldCoordinates(translation, rotation, hashtag.getCurrentTag(getID(i)));
        // System.out.println("X Value: " + robotPose.getX());
        // System.out.println("Z Value: " + robotPose.getY());
        return robotPose;
    }



    public Pose2d poseCalc(double x, double y, double rads){
        Translation2d translation = new Translation2d(x, y);
        Rotation2d rotation = new Rotation2d(rads);
        Pose2d pose = new Pose2d(translation, rotation);
        return pose;
      }
    
    public String stringtoVal(String str){
        // Switch to new method later since using NetworkTables3
        String str2 = "";
        String str3 = " ";
        char c = str3.charAt(0);
        for(int i = 0; i<str.length(); i++){
            if(str.charAt(i) != c){
                str2 = str2 + str.charAt(i);
            } else {
                break;
            }
        }
        return str2;
    
    }
    
    public String stringto2ndVal(String str){
        String str2 = "";
        String str3 = " ";
        char c = str3.charAt(0);
        boolean secondtrue = false;
        for(int i = 0; i<str.length(); i++){
            if(str.charAt(i) == c){
                secondtrue = true;
            }

            if(secondtrue){
                if(str.charAt(i) != c){
                    str2 = str2 + str.charAt(i);
                } else {
                    break;      
                }
                
            }
        }
        return str2;
    
    }

    public static Pose2d toFieldCoordinates(Translation2d translation2d, Rotation2d rotation2d, TestAprilTag apriltag) {
        Transform2d robotRelative = new Transform2d(translation2d, rotation2d);
        Transform2d tagRelative = robotRelative.inverse();
        Pose2d fieldRelative = apriltag.getPose().plus(tagRelative);
        return fieldRelative;
    }





}
