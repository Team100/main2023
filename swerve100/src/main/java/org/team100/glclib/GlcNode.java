package org.team100.glclib;

import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;

/**
 * Node object used in motion planning search tree
 * 
 * The Node object in the glc library will always belong to a motion
 * planning search tree. Each Node contains pointers to its parent
 * and children in the tree. A Node is also associated to a state in
 * the state space as well as a cost to reach that Node via the path
 * from the root of the tree. The node also has an attribute "merit"
 * which is the cost from the root together with the estimated
 * cost-to-go which is used in the A*-like search.
 */
public class GlcNode {
    /**
     * The pointer to this node's parent
     */
    public final GlcNode parent;
    /**
     * The trajectory from the parent node to this node
     */
    public final InterpolatingPolynomial trajectory_from_parent;
    /**
     * The control signal producing the trajectory from the parent node to
     * this node
     */
    public final InterpolatingPolynomial control_from_parent;
    /**
     * An array of pointers to this node's children
     * Vector< Node> children;
     */
    /**
     * The state or configuration associated with this node
     */
    public final double[] state;
    /**
     * The duration of a trajectory from the root of the search tree to the
     * state of this node
     */
    public final double time;
    /**
     * The cost to reach this node from the root
     */
    public final double cost;
    /**
     * The cost together with the estimated cost-to-go which is used for an
     * informed search
     */
    public final double merit;
    /**
     * The index from the discrete control set used to reach this node from
     * the parent in the current instance of the algorithm
     */
    public final int u_idx;
    /**
     * The depth of this node in the search tree
     * TODO: This is never updated
     */
    public int depth;
    /**
     * A flag to indicate if this Node is in the goal set
     */
    // private final boolean in_goal = false;

    /**
     * Constructor for the Node object which sets several of the struct's
     * attributes
     * 
     * @param card_omega_    the cardinality of the discrete set of controls
     *                       used in this instance of the algorithm
     * @param control_index_ the index of the control used to reach this
     *                       node's state from the its parent
     * @param cost_          the cost to reach this node from the root
     * @param cost_to_go_    an underestimate of the remaining cost-to-go from
     *                       the state of this node
     * @param state_         a the state of the system associated with this Node
     * @param time_          the duration of the trajectory from the root to this
     *                       node
     *                       in the search tree
     * @param parent_        a pointer to the parent of this Node
     */

    public GlcNode(int _card_omega,
            int _control_index,
            double _cost,
            double _cost_to_go,
            final double[] _state,
            double _time,
            final GlcNode _parent,
            final InterpolatingPolynomial _trajectory_from_parent,
            final InterpolatingPolynomial _control_from_parent) {
        // children = _card_omega),
        cost = _cost;
        merit = _cost_to_go + _cost;
        time = _time;
        parent = _parent;
        state = _state;
        u_idx = _control_index;
        control_from_parent = _control_from_parent;
        trajectory_from_parent = _trajectory_from_parent;
        if (parent != null) {
            if (parent.state[0] != trajectory_from_parent.at(parent.time)[0])
                throw new IllegalArgumentException();
            if (parent.state[1] != trajectory_from_parent.at(parent.time)[1])
                throw new IllegalArgumentException();
        }
    }

}
