package ctre_shims;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TalonAngleTest {
    public static final double DELTA = 1e-2; // for approx-equal

    @Test
    public void testToyCorrection() {
        assertEquals(0, TalonAngle.correctAndWrapDistance(0, 0, 1, 1), DELTA);
        assertEquals(0.5, TalonAngle.correctAndWrapDistance(0, 0.5, 1, 1), DELTA);
        assertEquals(2, TalonAngle.correctAndWrapDistance(1, 0, 1, 2), DELTA);
    }

    @Test
    public void testRealCorrection() {
        // real examples
        double inputRange = 893;
        double inputZero = 838;
        double outputRange = 2*Math.PI;
        assertEquals(2*Math.PI,
            TalonAngle.correctAndWrapDistance(837, inputZero, inputRange, outputRange),
            DELTA);
        assertEquals(0,
            TalonAngle.correctAndWrapDistance(839, inputZero, inputRange, outputRange),
            DELTA);
        assertEquals(Math.PI,
            TalonAngle.correctAndWrapDistance(392, inputZero, inputRange, outputRange),
            DELTA);
        assertEquals(Math.PI/2,
            TalonAngle.correctAndWrapDistance(168, inputZero, inputRange, outputRange),
            DELTA);

    }

    @Test
    public void testOutOfBounds() {
        double inputRange = 893;
        double inputZero = 838;
        double outputRange = 2*Math.PI;
        assertEquals(0,
            TalonAngle.correctAndWrapDistance(-54, inputZero, inputRange, outputRange),
            DELTA);
        assertEquals(Math.PI,
            TalonAngle.correctAndWrapDistance(1285, inputZero, inputRange, outputRange),
            DELTA);
    }
}
