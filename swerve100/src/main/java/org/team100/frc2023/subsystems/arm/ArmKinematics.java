package org.team100.frc2023.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;

class ArmKinematics {
    private static final double kUpperArmLength = 0.92;
    private static final double kLowerArmLength = 0.93;

    /**
     * Calculates the position of the arm based on the angles of the segments
     * 
     * @param lowerArmAngle
     * @param upperArmAngle
     * @return Translation2d object containing the position of the arm
     */
    public static Translation2d getArmPosition(double lowerArmAngle, double upperArmAngle) {
        // upperArmAngle += lowerArmAngle;
        double upperArmX = kUpperArmLength * Math.cos(upperArmAngle);
        double upperArmY = kUpperArmLength * Math.sin(upperArmAngle);

        double lowerArmX = kLowerArmLength * Math.cos(lowerArmAngle);
        double lowerArmY = kLowerArmLength * Math.sin(lowerArmAngle);

        return new Translation2d(upperArmX + lowerArmX, upperArmY + lowerArmY);
    }

    /**
     * Calculates the angles the arm should be at to reach the setpoint.
     * Accounts for the fact that our segment angles are independent.
     * 
     * @param x
     * @param y
     * @return array containing the angles [lowerArmAngle, upperArmAngle]
     */
    static double[] algorithm2RIK(double x, double y) {
        double[] angles = new double[2];

        double c2 = (x * x + y * y - kUpperArmLength * kUpperArmLength - kLowerArmLength * kLowerArmLength)
                / (2 * kUpperArmLength * kLowerArmLength);
        double s2 = Math.sqrt(1 - c2 * c2);
        angles[1] = Math.atan2(s2, c2);

        double k1 = kUpperArmLength + kLowerArmLength * c2;
        double k2 = kLowerArmLength * s2;
        angles[0] = Math.atan2(y, x) - Math.atan2(k2, k1);

        return angles;
    }

    static double[] algorithm2RIKS(double x, double y) {
        // https://www.youtube.com/watch?v=RH3iAmMsolo&t=7s

        double[] angles = new double[2];
        double xSquared = Math.pow(x, 2);
        double ySquared = Math.pow(y, 2);

        if (Math.sqrt(ySquared + xSquared) + 0.05 >= kLowerArmLength + kUpperArmLength) {
            return null;
        }

        double upperLengthSquare = Math.pow(kUpperArmLength, 2);
        double lowerLengthSquare = Math.pow(kLowerArmLength, 2);
        double lengthSquared = upperLengthSquare + lowerLengthSquare;
        double q2 = Math.PI
                - Math.acos((lengthSquared - xSquared - ySquared) / (2 * kUpperArmLength * kLowerArmLength));
        // maybe x/y try if it dosent work
        double q1 = Math.atan(y / x)
                - Math.atan((kUpperArmLength * Math.sin(q2)) / (kLowerArmLength + kUpperArmLength * Math.cos(q2)));

        angles[0] = q1 + q2; // upper theta
        angles[1] = q1; // lower theta

        return angles;

    }

}