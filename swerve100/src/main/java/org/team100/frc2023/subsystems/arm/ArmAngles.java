package org.team100.frc2023.subsystems.arm;

public class ArmAngles {
    public final double lowerTheta;
    public final double upperTheta;

    public ArmAngles(double upperTheta, double lowerTheta) {
        this.lowerTheta = lowerTheta;
        this.upperTheta = upperTheta;
    }

    public ArmAngles() {
        this.lowerTheta = 100;
        this.upperTheta = 100;
    }

}
