package org.team100.frc2023.kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinematics.ChassisSpeedFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedFactoryTest {
    private static final double kDelta = 0.01;

    /** This is identical to the test in WPI's ChassisSpeedsTest. */
    @Test
    void testFieldRelativeConstruction() {
        ChassisSpeedFactory factory = new ChassisSpeedFactory();
        final var chassisSpeeds = factory.fromFieldRelativeSpeeds(
                1.0, // vx
                0.0, // vy
                0.5, // omega
                Rotation2d.fromDegrees(-90.0) // robot angle
        );

        assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, kDelta);
        assertEquals(1.0, chassisSpeeds.vyMetersPerSecond, kDelta);
        assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testRotationCorrection() {
        ChassisSpeedFactory factory = new ChassisSpeedFactory(
                () -> 1.0, // observed omega rad/s
                0.2); // delay sec
        // 1 rad/s with 0.2s delay means 0.2 rad of error to correct.
        final var chassisSpeeds = factory.fromFieldRelativeSpeeds(
                1.0, // vx
                0.0, // vy
                0.5, // omega
                Rotation2d.fromDegrees(-90.0) // robot angle
        );

        // correction is +x-in-robot-frame, (-y-in-field-frame) to counteract the
        // lag-induced error, +y-in-field-frame
        assertEquals(0.20, chassisSpeeds.vxMetersPerSecond, kDelta); // sin 0.2
        assertEquals(0.98, chassisSpeeds.vyMetersPerSecond, kDelta); // cos 0.2
        assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testNegativeRotationCorrection() {
        ChassisSpeedFactory factory = new ChassisSpeedFactory(
                () -> -1.0, // observed omega rad/s
                0.2); // delay sec
        // 1 rad/s with 0.2s delay means 0.2 rad of error to correct.
        final var chassisSpeeds = factory.fromFieldRelativeSpeeds(
                1.0, // vx
                0.0, // vy
                0.5, // omega
                Rotation2d.fromDegrees(-90.0) // robot angle
        );

        // correction is -x-in-robot-frame, (+y-in-field-frame) to counteract the
        // lag-induced error, -y-in-field-frame

        assertEquals(-0.20, chassisSpeeds.vxMetersPerSecond, kDelta); // sin 0.2
        assertEquals(0.98, chassisSpeeds.vyMetersPerSecond, kDelta); // cos 0.2
        assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testCorrection() {
        ChassisSpeedFactory factory = new ChassisSpeedFactory(
                () -> 1.0, // observed omega rad/s
                0.2); // delay sec
        Rotation2d angle = factory.correctAngle(Rotation2d.fromDegrees(-90.0));
        assertEquals(-78.54, angle.getDegrees(), kDelta);
    }

    @Test
    void testInverse0() {
        ChassisSpeedFactory factory = new ChassisSpeedFactory(() -> 0, 0);
        ChassisSpeeds fieldRelative = factory.toFieldRelativeSpeeds(
                (double) 0,
                (double) 0,
                (double) 0,
                new Rotation2d());
        assertEquals(0, fieldRelative.vxMetersPerSecond, kDelta);
        assertEquals(0, fieldRelative.vyMetersPerSecond, kDelta);
        assertEquals(0, fieldRelative.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testInverse45() {
        ChassisSpeedFactory factory = new ChassisSpeedFactory(() -> 0, 0);
        ChassisSpeeds fieldRelative = factory.toFieldRelativeSpeeds(
                (double) 1,
                (double) 0,
                (double) 0,
                new Rotation2d(Math.PI/4));
        assertEquals(Math.sqrt(2) / 2, fieldRelative.vxMetersPerSecond, kDelta);
        assertEquals(Math.sqrt(2) / 2, fieldRelative.vyMetersPerSecond, kDelta);
        assertEquals(0, fieldRelative.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testInverse135Rot() {
        ChassisSpeedFactory factory = new ChassisSpeedFactory(() -> 0, 0);
        ChassisSpeeds fieldRelative = factory.toFieldRelativeSpeeds(
                (double) 0,
                (double) 1,
                (double) 1,
                new Rotation2d(3 * Math.PI/4));
        assertEquals(-1.0 * Math.sqrt(2) / 2, fieldRelative.vxMetersPerSecond, kDelta);
        assertEquals(-1.0 * Math.sqrt(2) / 2, fieldRelative.vyMetersPerSecond, kDelta);
        assertEquals(1, fieldRelative.omegaRadiansPerSecond, kDelta);
    }

}
