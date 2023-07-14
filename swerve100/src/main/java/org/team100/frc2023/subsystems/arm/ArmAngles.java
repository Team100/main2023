package org.team100.frc2023.subsystems.arm;

public class ArmAngles {
    /** pi is up, pi/2 is forward */
    public final double upperTheta;
    /** 0 is up, positive is forward */
    public final double lowerTheta;

    /**
     * @param upperTheta radians
     * @param lowerTheta radians
     */
    public ArmAngles(double upperTheta, double lowerTheta) {
        this.upperTheta = upperTheta;
        this.lowerTheta = lowerTheta;
    }

    public ArmAngles() {
        this.upperTheta = 100;
        this.lowerTheta = 100;
    }

}
