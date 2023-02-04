package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose3d;

public class TestAprilTag {
    
    public int ID;
    public Pose3d Pose3d;
    
    public TestAprilTag(int ID, Pose3d pose) {
        this.ID = ID;
        this.Pose3d = pose;
    }

    public Pose3d getPose(){
        return this.Pose3d;
    }

}
