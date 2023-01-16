package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionEstimate {
    private Pose2d estimatedPose;
    private double timestamp;
    public VisionEstimate(Pose2d estimatedPose, double timestamp) {
        this.estimatedPose = estimatedPose;
        this.timestamp = timestamp;
    }
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }
    public double getTimestamp() {
        return timestamp;
    }
}
