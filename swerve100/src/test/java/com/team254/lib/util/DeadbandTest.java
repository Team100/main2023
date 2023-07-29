package com.team254.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class DeadbandTest {

    private final double kEpsilon = 1e-5;

    @Test
    public void testInDeadband() {
        assertEquals(Util.handleDeadband(1, .5), 1, kEpsilon);
        assertEquals(Util.handleDeadband(.5, .1), 0.4444444444, kEpsilon);
        assertEquals(Util.handleDeadband(.1, .2), 0, kEpsilon);
        assertEquals(Util.handleDeadband(.8, .2), 0.75, kEpsilon);
    }
}
