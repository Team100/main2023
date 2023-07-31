package org.team100.glclib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.glclib.GlcNode;
import org.team100.glclib.NodeMeritOrder;

public class TestGlcNode {

    /**
     * This test checks the following properties of a Node:
     * 1) That the merit attribute is set to the some of cost with the estimated
     * cost-to-go
     * 2) That the merit of child is greater than root since root has less cost plus
     * heuristic
     * 3) That the parent attribute in child points to the root node
     */
    @Test
    public void testOrder() {
        GlcNode root = new GlcNode(5, 0, 10.0, 10.0, new double[] { 0.0, 1.0 }, 1.0, null, null, null);
        GlcNode child = new GlcNode(4, 1, 8.0, 14.0, new double[] { 1.0, 1.0 }, 1.0, null, null, null);

        NodeMeritOrder comparator = new NodeMeritOrder();

        assertEquals(root.merit, 20.0);
        assertEquals(child.merit, 22.0);
        assertTrue(comparator.compare(root, child) < 0);
        assertTrue(comparator.compare(child, root) > 0);
    }
}
