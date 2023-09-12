package com.team254.lib.util;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class StickyBooleanTest {
    @Test
    public void testLatches() {
        StickyBoolean b = new StickyBoolean();
        assertFalse(b.update(false));
        assertTrue(b.update(true));
        assertTrue(b.update(false));
        assertTrue(b.get());
    }

    @Test
    public void testReset() {
        StickyBoolean b = new StickyBoolean();
        assertTrue(b.update(true));
        b.reset();
        assertFalse(b.get());
        assertFalse(b.update(false));
    }
}
