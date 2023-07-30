package org.team100.glclib;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.glclib.GlcStateEquivalenceClass;

public class TestGlcEquivalenceRelation {

    /**
     * This checks that the elements of the partition of the state space are ordered
     * lexicographically by their multi-dimensional index.
     */
    @Test
    public void testOrder() {

        final GlcStateEquivalenceClass d0 = new GlcStateEquivalenceClass(new int[] { 1, 2, 3 });
        final GlcStateEquivalenceClass d1 = new GlcStateEquivalenceClass(new int[] { 1, 3, 3 });
        final GlcStateEquivalenceClass d2 = new GlcStateEquivalenceClass(new int[] { 1, 2, 3 });

        assertTrue(d0.compareTo(d1) < 0);
        assertFalse(d1.compareTo(d0) < 0);

        assertTrue(d2.compareTo(d1) < 0);
        assertFalse(d1.compareTo(d2) < 0);

        assertTrue(d2.compareTo(d0) == 0);
    }

}
