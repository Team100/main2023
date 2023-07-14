package org.team100.frc2023.kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.frc2023.subsystems.arm.ArmAngles;
import org.team100.frc2023.subsystems.arm.ArmKinematics;

import edu.wpi.first.math.geometry.Translation2d;

public class ArmKinematicsTest {
    private static final double kDelta = 0.001;

    @Test
    void testForward1() {
        ArmKinematics kinematics = new ArmKinematics(0.93, 0.92);// like 2023
        double lowerTheta = 0; // up
        double upperTheta = Math.PI / 2; // forward
        Translation2d position = kinematics.forward(lowerTheta, upperTheta);
        assertEquals(0.93, position.getX(), kDelta); // lower arm length
        assertEquals(0.92, position.getY(), kDelta); // upper arm length
    }

    @Test
    void testInverse1() {
        ArmKinematics kinematics = new ArmKinematics(0.93, 0.92);// like 2023
        double xM = 0.93;
        double yM = 0.92;
        ArmAngles angles = kinematics.inverse(xM, yM);
        assertEquals(0, angles.th1, kDelta);
        assertEquals(Math.PI / 2, angles.th2, kDelta);
    }

    @Test
    void testForward2() {
        ArmKinematics kinematics = new ArmKinematics(0.93, 0.92);// like 2023
        double lowerTheta = Math.PI / 4; // half forward
        double upperTheta = Math.PI / 2; // forward
        Translation2d position = kinematics.forward(lowerTheta, upperTheta);
        assertEquals(0.658, position.getX(), kDelta); // 0.93 * sqrt(2)/2
        assertEquals(1.578, position.getY(), kDelta); // 0.92 + 0.93 * sqrt(2)/2
    }

    @Test
    void testInverse2() {
        ArmKinematics kinematics = new ArmKinematics(0.93, 0.92);// like 2023
        double xM = 0.658;
        double yM = 1.578;
        ArmAngles angles = kinematics.inverse(xM, yM);
        assertEquals(Math.PI / 4, angles.th1, kDelta);
        assertEquals(Math.PI / 2, angles.th2, kDelta);
    }

    @Test
    void testForward3() {
        ArmKinematics kinematics = new ArmKinematics(0.93, 0.92); // like 2023
        double lowerTheta = Math.PI / 4; // half forward
        double upperTheta = Math.PI / 4; // half forward
        Translation2d position = kinematics.forward(lowerTheta, upperTheta);
        assertEquals(1.308, position.getX(), kDelta); // (0.92 + 0.93) * sqrt(2)/2
        assertEquals(1.308, position.getY(), kDelta); // (0.92 + 0.93) * sqrt(2)/2
    }

    @Test
    void testInverse3() {
        ArmKinematics kinematics = new ArmKinematics(0.93, 0.92);// like 2023
        double xM = 1.308147;
        double yM = 1.308147;
        ArmAngles angles = kinematics.inverse(xM, yM);
        // the angles here are very sensitive because the elbow is fully extended.
        assertEquals(Math.PI / 4, angles.th1, kDelta);
        assertEquals(Math.PI / 4, angles.th2, kDelta);
    }

    @Test
    void testForward4() {
        ArmKinematics kinematics = new ArmKinematics(1, 1); // unit length
        double lowerTheta = 0; // up
        double upperTheta = Math.PI; // down
        Translation2d position = kinematics.forward(lowerTheta, upperTheta);
        assertEquals(0, position.getX(), kDelta);
        assertEquals(0, position.getY(), kDelta);
    }

    @Test
    void testInverse4() {
        ArmKinematics kinematics = new ArmKinematics(1, 1); // unit length
        double xM = 0;
        double yM = 0;
        ArmAngles angles = kinematics.inverse(xM, yM);
        assertNull(angles); // this case is underdetermined
    }

    @Test
    void testForward5() {
        ArmKinematics kinematics = new ArmKinematics(1, 1); // unit length
        double lowerTheta = Math.PI / 4; // half forward
        double upperTheta = 3 * Math.PI / 4; // down diagonal
        Translation2d position = kinematics.forward(lowerTheta, upperTheta);
        assertEquals(0, position.getX(), kDelta);
        assertEquals(1.414, position.getY(), kDelta); // sqrt(2)
    }

    @Test
    void testInverse5() {
        ArmKinematics kinematics = new ArmKinematics(1, 1);// unit length
        double xM = 0;
        double yM = 1.414;// sqrt(2)
        ArmAngles angles = kinematics.inverse(xM, yM);
        assertEquals(Math.PI / 4, angles.th1, kDelta);
        assertEquals(3 * Math.PI / 4, angles.th2, kDelta);
    }

    @Test
    void testInverseUnreachable() {
        ArmKinematics kinematics = new ArmKinematics(1, 1); // unit length
        double xM = 2;
        double yM = 2;
        ArmAngles angles = kinematics.inverse(xM, yM);
        assertNull(angles); // this case is unreachable
    }
}
