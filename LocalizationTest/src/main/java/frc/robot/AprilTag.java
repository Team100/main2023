package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class AprilTag {
    
    public int ID;
    public Pose2d Pose2d;
    
    public AprilTag(int ID, Pose2d pose) {
        this.ID = ID;
        this.Pose2d = pose;
    }

    public Pose2d getPose() {
        return this.Pose2d;
    }

}
