package frc.robot;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.robot.subsystems.TalonSRXTurningEncoder;

public class TalonSRXTurningEncoderTest {
    public static final double DELTA = 1e-2; // for approx-equal

    @Test
    public void testToyCorrection() {
        assertEquals(0, TalonSRXTurningEncoder.correctAndWrapAngle(0, 0, 1, 1), DELTA);
        assertEquals(0.5, TalonSRXTurningEncoder.correctAndWrapAngle(0, 0.5, 1, 1), DELTA);
        assertEquals(2, TalonSRXTurningEncoder.correctAndWrapAngle(1, 0, 1, 2), DELTA);
    }

    @Test
    public void testRealCorrection() {
        // real examples
        double inputRange = 893;
        double inputZero = 838;
        double outputRange = 2 * Math.PI;
        assertEquals(2 * Math.PI,
                TalonSRXTurningEncoder.correctAndWrapAngle(837, inputZero, inputRange, outputRange),
                DELTA);
        assertEquals(0,
                TalonSRXTurningEncoder.correctAndWrapAngle(839, inputZero, inputRange, outputRange),
                DELTA);
        assertEquals(Math.PI,
                TalonSRXTurningEncoder.correctAndWrapAngle(392, inputZero, inputRange, outputRange),
                DELTA);
        assertEquals(Math.PI / 2,
                TalonSRXTurningEncoder.correctAndWrapAngle(168, inputZero, inputRange, outputRange),
                DELTA);
    }

    @Test
    public void testOutOfBounds() {
        double inputRange = 893;
        double inputZero = 838;
        double outputRange = 2 * Math.PI;
        assertEquals(0,
                TalonSRXTurningEncoder.correctAndWrapAngle(-54, inputZero, inputRange, outputRange),
                DELTA);
        assertEquals(Math.PI,
                TalonSRXTurningEncoder.correctAndWrapAngle(1285, inputZero, inputRange, outputRange),
                DELTA);
    }
}