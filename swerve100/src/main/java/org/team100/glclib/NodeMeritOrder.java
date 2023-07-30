package org.team100.glclib;

import java.util.Comparator;

/**
 * A structure for defining a weak linear ordering of nodes based on the
 * ordering of their merit attribute.
 *
 * This ordering is used to order a priority queue used in the A* like search.
 */
public class NodeMeritOrder implements Comparator<GlcNode> {

    /**
     * Compare nodes based on merit.
     * 
     * @param l the left element to be checked for membership in the relation
     * @param r the right element to be checked for membership in the
     *              relation
     * @returns negative if l < r, positive if l > r, zero otherwise
     */

    @Override
    public int compare(GlcNode l, GlcNode r) {
        return Double.compare(l.merit, r.merit);
    }
}