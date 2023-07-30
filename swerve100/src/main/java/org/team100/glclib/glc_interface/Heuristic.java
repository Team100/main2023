package org.team100.glclib.glc_interface;

/**
 * Base class for an admissible and consistent estimate of the cost-to-go
 * from a particular state in the state space
 * 
 * The heuristic is computed at the state associated to nodes in the search tree
 * with the ordering of nodes in a prority queue determined by the cost to reach
 * that node from the root of the tree together with the estimated cost-to-go.
 */
public abstract class Heuristic {

    /**
     * Base class for evaluates of the under-estimates the cost to go from a
     * particular state
     * 
     * The the estimated cost-to-go should be as close as possible to the actual
     * cost-to-go without over-approximating it anywhere in the state space.
     * Nominally, the heuristic is computationally cheap to evaluate.
     * 
     * @param x0_ is the state from which the cos-to-go is estimated
     * @return estimated cost to go
     */
    public abstract double costToGo(final double[] x0_);
};