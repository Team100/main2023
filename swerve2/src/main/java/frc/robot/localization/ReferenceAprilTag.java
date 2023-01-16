package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;

public class ReferenceAprilTag {
    
    public int ID;
    public Pose2d Pose2d;
    
    public ReferenceAprilTag(int ID, Pose2d pose) {
        this.ID = ID;
        this.Pose2d = pose;
    }

    public Pose2d getPose(){
        return this.Pose2d;
    }

}