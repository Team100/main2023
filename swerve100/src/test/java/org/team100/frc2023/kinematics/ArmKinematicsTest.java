package org.team100.frc2023.kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.frc2023.subsystems.arm.ArmAngles;
import org.team100.frc2023.subsystems.arm.ArmKinematics;

import edu.wpi.first.math.geometry.Translation2d;

public class ArmKinematicsTest {
    private static final double kDelta = 0.01;

    @Test
    void testForward1() {
        double lowerTheta = 0; // up
        double upperTheta = Math.PI / 2; // forward
        Translation2d position = ArmKinematics.forward(lowerTheta, upperTheta);
        assertEquals(0.93, position.getX(), kDelta); // lower arm length
        assertEquals(0.92, position.getY(), kDelta); // upper arm length
    }

    @Test
    void testForward2() {
        double lowerTheta = Math.PI / 4; // half forward
        double upperTheta = Math.PI / 2; // forward
        Translation2d position = ArmKinematics.forward(lowerTheta, upperTheta);
        assertEquals(0.658, position.getX(), kDelta); // 0.93 * sqrt(2)/2
        assertEquals(1.578, position.getY(), kDelta); // 0.92 + 0.93 * sqrt(2)/2

    }

    @Test
    void testForward3() {
        double lowerTheta = Math.PI / 4; // half forward
        double upperTheta = Math.PI / 4; // half forward
        Translation2d position = ArmKinematics.forward(lowerTheta, upperTheta);
        assertEquals(1.308, position.getX(), kDelta); // (0.92 + 0.93) * sqrt(2)/2
        assertEquals(1.308, position.getY(), kDelta); // (0.92 + 0.93) * sqrt(2)/2

    }

    @Test
    void testInverse1() {
        double xM = 0.93;
        double yM = 0.92;
        ArmAngles angles = ArmKinematics.inverse(xM, yM);
        assertEquals(0, angles.lowerTheta, kDelta);
        assertEquals(Math.PI / 2, angles.upperTheta, kDelta);
    }

    @Test
    void testInverse2() {
        double xM = 0.658;
        double yM = 1.578;
        ArmAngles angles = ArmKinematics.inverse(xM, yM);
        assertEquals(Math.PI / 4, angles.lowerTheta, kDelta);
        assertEquals(Math.PI / 2, angles.upperTheta, kDelta);
    }

    @Test
    void testInverse3() {
        double xM = 1.308147;
        double yM = 1.308147;
        ArmAngles angles = ArmKinematics.inverse(xM, yM);
        // the angles here are very sensitive because the elbow is fully extended.
        assertEquals(Math.PI / 4, angles.lowerTheta, kDelta);
        assertEquals(Math.PI / 4, angles.upperTheta, kDelta);
    }
}
