package frc.robot.localization;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisonProvider {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");

    double[][] defaultValue;
    String[][] strDefaultValue;

    double[] defauValue;


    public VisonProvider(){
        NetworkTableEntry pose_t_x =  table.getEntry("pose_t_x");
        NetworkTableEntry pose_t_y =  table.getEntry("pose_t_y");
        NetworkTableEntry pose_t_z =  table.getEntry("pose_t_z");
        NetworkTableEntry pose_R_x =  table.getEntry("pose_R_x");
        NetworkTableEntry pose_R_y =  table.getEntry("pose_R_y");
        NetworkTableEntry pose_R_z =  table.getEntry("pose_R_z");
    }

    public boolean aprilPresent(){
        
    }

    public void periodic(){
       

        double [][] yValues;
        double [][] zValues;

    }
    
}
