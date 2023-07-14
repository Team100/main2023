package org.team100.frc2023.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Coordinates are as follows:
 * * x up
 * * y forward
 * * joint angles absolute, relative to x
 */
public class ArmKinematics {
    private static final double kUpperArmLengthM = 0.92;
    private static final double kLowerArmLengthM = 0.93;

    /**
     * Calculates the position of the arm based on absolute joint angles.
     * 
     * @param lowerArmAngle radians
     * @param upperArmAngle radians
     * @return end position in meters
     */
    public static Translation2d forward(double lowerArmAngle, double upperArmAngle) {
        double upperArmX = kUpperArmLengthM * Math.cos(upperArmAngle);
        double upperArmY = kUpperArmLengthM * Math.sin(upperArmAngle);

        double lowerArmX = kLowerArmLengthM * Math.cos(lowerArmAngle);
        double lowerArmY = kLowerArmLengthM * Math.sin(lowerArmAngle);

        return new Translation2d(upperArmX + lowerArmX, upperArmY + lowerArmY);
    }

    /**
     * Calculate absolute joint angles given cartesian coords of the end.
     * 
     * @param x meters
     * @param y meters
     * @return absolute joint angles
     */
    public static ArmAngles inverse(double x, double y) {
        double r = Math.sqrt(x * x + y * y);
        double l = kLowerArmLengthM;
        double u = kUpperArmLengthM;
        double thetaLower = Math.atan2(y, x) - Math.acos((r * r + l * l - u * u) / (2 * r * l));
        double thetaUpper = Math.PI + thetaLower - Math.acos((l * l + u * u - r * r) / (2 * l * u));
        return new ArmAngles(thetaUpper, thetaLower);
    }
}