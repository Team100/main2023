package org.team100.glclib;

import java.util.Arrays;
import java.util.PriorityQueue;

/**
 * This class defines the equivalence classes (i.e. partition) of the state
 * space
 * 
 * The GLC algorithm is able to build a sparse search tree by maintaining at
 * most one node per equivalence class. The size of the equivalence class is
 * determined by the resolution parameter in the current planning query.
 */
class GlcStateEquivalenceClass implements Comparable<GlcStateEquivalenceClass> {
    // Each equivalence class (a hyper-cubicle region) is uniquely identified by an
    // integer tuple
    final int[] coordinate;

    // An equivalence class has a pointer to a node whose associated state is in the
    // cubicle region. Note final because, duh, it gets corrected.
    GlcNode label;

    /**
     * A priority queue of potential new nodes that could label the cell
     * 
     * Once relabeled, the subtree rooted at the old label is deleted.
     */
    final PriorityQueue<GlcNode> candidates = new PriorityQueue<GlcNode>(new NodeMeritOrder());

    public GlcStateEquivalenceClass(int[] coordinate, GlcNode label) {
        this.coordinate = coordinate;
        this.label = label;
    }

    public GlcStateEquivalenceClass(int[] coordinate) {
        this(coordinate, new GlcNode(0,
                -1,
                Double.MAX_VALUE,
                Double.MAX_VALUE,
                new double[0],
                0,
                null,
                null,
                null));

    }

    // If no label has been set and the label attribute is null, then empty will
    // return true -- and false otherwise
    boolean empty() {
        return label.cost == Double.MAX_VALUE;
    }

    // // Lexicographical ordering is used to support std::map
    // boolean operator<(
    // final GlcStateEquivalenceClass y)
    // {
    // if (coordinate.size() != y.coordinate.size()) throw new
    // IllegalArgumentException();

    // return std::lexicographical_compare <Vector<int>::const_iterator,
    // Vector<int>::const_iterator>
    // (coordinate.begin(), coordinate.end(), y.coordinate.begin(),
    // y.coordinate.end());
    // }
    @Override
    public int compareTo(GlcStateEquivalenceClass y) {
        return Arrays.compare( coordinate, y.coordinate);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        GlcStateEquivalenceClass other = (GlcStateEquivalenceClass) obj;
        if (coordinate == null) {
            if (other.coordinate != null)
                return false;
        } else if (!Arrays.equals(coordinate, other.coordinate))
            return false;
        return true;
    }

}
