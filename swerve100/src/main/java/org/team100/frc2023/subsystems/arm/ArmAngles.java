package org.team100.frc2023.subsystems.arm;

/**
 * Represents a 2DOF serial arm.
 */
public class ArmAngles {
    public final double th1;
    public final double th2;

    /**
     * Theta counting out from the grounded joint.
     * 
     * @param th1 shoulder radians
     * @param th2 elbow radians
     */
    public ArmAngles(double th1, double th2) {
        this.th1 = th1;
        this.th2 = th2;
    }
}
