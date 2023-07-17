package com.team254.lib.motion;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class MotionSegmentTest {

    @Test
    public void valid() {
        MotionSegment a = new MotionSegment(new MotionState(0.0, 0.0, 0.0, 0.0), new MotionState(0.0, 0.0, 0.0, 1.0));
        assertFalse(a.isValid());

        a = new MotionSegment(new MotionState(0.0, 0.0, 0.0, 1.0), new MotionState(1.0, 0.0, 1.0, 1.0));
        assertFalse(a.isValid());

        a = new MotionSegment(new MotionState(0.0, 0.0, 0.0, 1.0), new MotionState(1.0, .5, 1.0, 1.0));
        assertTrue(a.isValid());
    }

}
