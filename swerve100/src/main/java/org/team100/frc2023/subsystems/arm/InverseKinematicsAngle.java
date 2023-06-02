package org.team100.frc2023.subsystems.arm;

/** Add your docs here. */
public class InverseKinematicsAngle {
    public double lowerTheta;
    public double upperTheta;

    public InverseKinematicsAngle(double upperTheta, double lowerTheta) {
        this.lowerTheta = lowerTheta;
        this.upperTheta = upperTheta;
    }

    
    public InverseKinematicsAngle() {
        this.lowerTheta = 100;
        this.upperTheta = 100;
    }

}
