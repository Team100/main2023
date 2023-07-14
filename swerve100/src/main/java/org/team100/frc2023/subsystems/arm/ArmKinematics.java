package org.team100.frc2023.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;

public class ArmKinematics {
    private final double l1;
    private final double l2;

    /**
     * Lengths counting out from the grounded joint.  Units here determine units below.
     * 
     * @param l1 proximal
     * @param l2 distal
     */
    public ArmKinematics(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
    }

    /**
     * Calculates the position of the arm based on absolute joint angles, counting
     * out from the grounded joint.
     * 
     * @param th1 absolute proximal radians
     * @param th2 absolute distal radians
     * @return end position
     */
    public Translation2d forward(double th1, double th2) {
        return new Translation2d(
                l1 * Math.cos(th1) + l2 * Math.cos(th2),
                l1 * Math.sin(th1) + l2 * Math.sin(th2));
    }

    /**
     * Calculate absolute joint angles given cartesian coords of the end.
     * 
     * @param x
     * @param y
     * @return absolute joint angles, null if unreachable.
     */
    public ArmAngles inverse(double x, double y) {
        double r = Math.sqrt(x * x + y * y);
        double th1 = Math.atan2(y, x) - Math.acos((r * r + l1 * l1 - l2 * l2) / (2 * r * l1));
        double th2 = Math.PI + th1 - Math.acos((l1 * l1 + l2 * l2 - r * r) / (2 * l1 * l2));
        if (Double.isNaN(th1) || Double.isNaN(th2))
            return null;
        return new ArmAngles(th1, th2);
    }
}