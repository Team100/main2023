import static org.junit.Assert.*;

import java.util.HashMap;

import org.junit.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.PoseTest;
import frc.robot.TestAprilTag;


public class TagTest {
    // @Test


    //put in robotPeriodic as a method??

    public void coordinatesTest() {
    
        TestAprilTag currentTag;
        Translation2d currentTranslation2d;
        Rotation2d currentRotation2d;
        Pose2d robotPose;
        double pastPosX = 0;
        double pastPosZ = 0;
        double pastRotX = 0;
        double pastRotZ = 0;
        double currentRobotX = 0;
        double currentRobotZ = 0;
        double currentRobotRot = 0;

        NetworkTable table;
        double[] defaultValue = new double[0];
        String[] strDefaultValue = new String[0];
        
        TestAprilTag tag1 = new TestAprilTag(1, poseCalc(1, 1, -Math.PI/2));
        TestAprilTag tag2 = new TestAprilTag(1, poseCalc(2, 1, -Math.PI/4));

        HashMap <Integer, TestAprilTag> aprilHash = new HashMap<Integer, TestAprilTag>();
        aprilHash.put(1, tag1);
        aprilHash.put(2, tag2);

        table = NetworkTableInstance.getDefault().getTable("Vision");
                double [] idValues = table.getEntry("idVals").getDoubleArray(defaultValue);
                double [] xValues = table.getEntry("pose_t_x").getDoubleArray(defaultValue);
                double [] yValues = table.getEntry("pose_t_y").getDoubleArray(defaultValue);
                double [] zValues = table.getEntry("pose_t_z").getDoubleArray(defaultValue);
                String [] xRot = table.getEntry("rot_t_x").getStringArray(strDefaultValue)
                String [] yRot = table.getEntry("rot_t_x").getStringArray(strDefaultValue);
                String [] zRot = table.getEntry("rot_t_x").getStringArray(strDefaultValue);
        
        
        //add giant if statement right here? if there isnt a val for ALL of them or if detected is true 0


        
        System.out.println("idValues: " + idValues);

        currentTag = aprilHash.get((int)idValues[0]); //only if there is an id tag


        currentTranslation2d = new Translation2d(xValues.length>0?xValues[0]:pastPosX, zValues.length>0?zValues[0]:pastPosZ);
        currentRotation2d = new Rotation2d(xRot.length>0&&zRot.length>0?Math.sin(xRot[0]/zRot[0]):Math.sin(pastRotX/pastRotZ));
        robotPose = PoseTest.toFieldCoordinates(currentTranslation2d, currentRotation2d, currentTag);

        currentRobotX = robotPose.getTranslation().getX();
        currentRobotZ = robotPose.getTranslation().getY();
        currentRobotRot = robotPose.getRotation().getRadians();    
        
        System.out.println("X: " + currentRobotX);
        System.out.println("Z: " + currentRobotZ);
        System.out.println("RADIANS: " + currentRobotRot);




        // if(xValues.length > 0){
        //     pastPosX = xValues[0];
        // }
        // if(zValues.length > 0){
        //     pastPosZ = zValues[0];
        // }
        // if(xRot.length > 0){
        //     pastRotX = xRot[0];
        // }
        // if(zRot.length > 0){
        //     pastRotZ = zRot[0];
        // }




        




        //fake data
        // Translation2d translation2d = new Translation2d(0, 1.0/2);
        // Rotation2d rotation2d = new Rotation2d(0);

        // //april data
        // Translation2d apriltranslation2d = new Translation2d(1, 1);
        // Rotation2d aprilrotation2d = new Rotation2d(-Math.PI/2);
        // Pose2d aprilPose = new Pose2d(apriltranslation2d, aprilrotation2d);
        // TestAprilTag aprilTag = new TestAprilTag(1, aprilPose);

        // //expected data
        // Translation2d expectedt = new Translation2d(1.0/2, 1);
        // Rotation2d expectedr = new Rotation2d(-Math.PI/2);
        // Pose2d expectedTransform = new Pose2d(expectedt, expectedr);

        // assertEquals(expectedTransform, PoseTest.toFieldCoordinates(translation2d, rotation2d, aprilTag));
    }

    public Pose2d poseCalc(int x, int y, double rads){
        Translation2d aprilTranslation2d = new Translation2d(x, y);
        Rotation2d aprilRotation2d = new Rotation2d(rads);
        Pose2d aprilPose = new Pose2d(aprilTranslation2d, aprilRotation2d);
        return aprilPose;
    }

    public String stringtoVal(String str){
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
}
