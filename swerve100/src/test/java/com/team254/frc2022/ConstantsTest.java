package com.team254.frc2022;

import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

/**
 * Ensure we don't push bad constants
 */
public class ConstantsTest {
    @Test
    public void test() {
        assertFalse( Constants.kForceDriveGamepad);
    }
}