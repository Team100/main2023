package org.team100.glclib;

import java.util.Collections;
import java.util.Iterator;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.Vector;

import org.team100.glclib.glc_interface.CostFunction;
import org.team100.glclib.glc_interface.DynamicalSystem;
import org.team100.glclib.glc_interface.GoalRegion;
import org.team100.glclib.glc_interface.Heuristic;
import org.team100.glclib.glc_interface.Obstacles;
import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;

/**
 * The core planning class for carring out trajectory optimization
 * 
 * The algorithms is simply an A* search. The innovation is in the
 * construction of a sparse, and nearly optimal discrete abstraction
 * of the set of feasible trajectories for a problem instance.
 */
public class Planner {
    private static final float CLOCKS_PER_SEC = 1000;
    /** A leaf node in the goal region with minimum cost from the root */
    GlcNode best;
    /** The root of the search tree */
    private final GlcNode root_ptr;
    /** A raw pointer to the dynamical system for the problem */
    private final DynamicalSystem dynamics;
    /** A raw pointer to the goal region for the problem */
    private final GoalRegion goal;
    /** A raw pointer to the (topologically)closed obstacle set for the problem */
    private final Obstacles obs;
    /** A raw pointer to the cost function for the problem */
    private final CostFunction cf;
    /** A raw pointer to the heuristic for the problem */
    private final Heuristic h;
    /** A comparator for measureing relative merit of nodes */
    private final NodeMeritOrder compare = new NodeMeritOrder();
    /**
     * A priority queue of leaf nodes that are open for expansion
     * 
     * The queue is ordered by the cost plus the estimated cost to go
     * determined by the admissible and consistent heuristic.
     */
    private final PriorityQueue<GlcNode> queue = new PriorityQueue<GlcNode>(new NodeMeritOrder());

    /**
     * A constant factor multiplying the depth limit of the search tree
     * 
     * In order to guarantee finite time termination, the algorithm is limited
     * to a finite search depth for a fixed search resolution. This limit is
     * increased with increasing resolution at a carefully controlled rate.
     * The user is free to tune a constant factor.
     */
    private final int depth_limit;
    /**
     * A constant factor dilation of the equivalence class size
     *
     * The partition size is carefully controlled by the resolution
     * parameter. It will decrease as the resolution increases, and
     * with larger Lipschitz coefficients in the differential
     * constraint the faster the partition must shrink to guaranteed
     * convergence. partition_scale is a constant factor multiplying
     * this function of the Lipschitz constant.
     */
    // this one is not used; the one in the parameters object is.
    // private final double partition_scale;

    /**
     * The side length of the cubicle's forming the partition of the state space
     * 
     * This value is a function of resolution determined
     * in the constructor. In the paper this value is eta(R).
     */
    private final double inverse_cubicle_side_length;
    /**
     * When a node is expanded, each this is the duration of the trajectories to the
     * child nodes
     */
    private final double expand_time;
    /**
     * This is used as a flag to stop expanding nodes and clear the priority queue
     * to find the best solution
     */
    private boolean found_goal = false;
    /**
     * This is a flag to stop the algorithm if some limit has been reached
     */
    private boolean live = true;
    /**
     * a copy of the struct containing planner parameters
     */
    private final GlcParameters params;
    /**
     * An array containing the discrete set of control inputs
     * 
     * When a node is expanded each of these control inputs
     * is applied to the system dynamics from the state associated
     * with the node being expanded for a duration of
     * expand_time.
     */
    private final Vector<double[]> controls;
    // A counter for the number of calls to the sim method
    // private final int sim_count = 0;
    // A counter for the number of calls to collisionFree
    // private final int coll_check = 0;
    // A counter for the number of calls to the expand method
    private int iter = 0;
    // A counter for the number of clock cycles used in a query
    private long run_time, tstart;

    /**
     * An ordered set of equivalence classes that have been reached by a trajectory
     * from the initial state
     * 
     * The labels on each equivalence class is the node
     * with best known merit in that equivalence class.
     */
    public SortedMap<GlcStateEquivalenceClass, GlcStateEquivalenceClass> partition_labels = new TreeMap<GlcStateEquivalenceClass, GlcStateEquivalenceClass>();
    Iterator<GlcStateEquivalenceClass> it;

    /**
     * The constructor assigns its parameters to the associated member attributes
     * 
     * The essential scaling functions of the method are evaluated at the given
     * resolution within the constructor.
     */
    public Planner(Obstacles _obs,
            GoalRegion _goal,
            DynamicalSystem _dynamics,
            Heuristic _h,
            CostFunction _cf,
            final GlcParameters _params,
            final Vector<double[]> _controls) {

        params = _params;
        controls = _controls;
        dynamics = _dynamics;
        obs = _obs;
        goal = _goal;
        cf = _cf;
        h = _h;
        root_ptr = new GlcNode(_controls.size(), 0, 0, _h.costToGo(params.x0), params.x0, 0, null, null, null);
        best = new GlcNode(0, -1, Double.MAX_VALUE, Double.MAX_VALUE, new double[0], 0, null, null, null);
        //////////// *Scaling functions*//////////////
        // 1/R
        expand_time = params.time_scale / (double) params.res;
        // h(R)
        depth_limit = params.depth_scale * params.res * (int) Math.floor(Math.log(params.res));
        // eta(R) \in \little-omega (log(R)*R^L_f)
        if (dynamics.getLipschitzConstant() == 0.0) {
            inverse_cubicle_side_length = params.res * Math.log(params.res) * Math.log(params.res)
                    / params.partition_scale;
        } else {
            inverse_cubicle_side_length = Math.pow(params.res, 1 + dynamics.getLipschitzConstant())
                    / params.partition_scale;
        }

        double[] inp = new double[root_ptr.state.length];
        for (int j = 0; j < inp.length; ++j) {
            inp[j] = inverse_cubicle_side_length * root_ptr.state[j];
        }

        GlcStateEquivalenceClass d0 = new GlcStateEquivalenceClass(GlcMath.vecFloor(inp), root_ptr);

        queue.add(root_ptr);
        partition_labels.put(d0, d0);

        // Print a summary of the algorithm parameters
        System.out.println("\n\n\n\nPre-search summary:\n");
        System.out.println("      Expand time: " + expand_time);
        System.out.println("      Depth limit: " + depth_limit);
        System.out.println("   Partition size: " + 1.0 / inverse_cubicle_side_length);
        System.out.println("   Max iterations: " + params.max_iter);

        // tstart = clock();
        tstart = System.currentTimeMillis();
    }

    /**
     * This method returns the sequence of nodes from the lowest cost node in the
     * goal back to the root via edge relations stored in each node
     * 
     * @param forward a flag to indicate if the nodes should be ordered from root to
     *                leaf (foward=true) or from leaf to root (forward=false)
     */

    public Vector<GlcNode> pathToRoot(boolean forward) {
        if (found_goal == false) {
            Vector<GlcNode> empty_vector = new Vector<GlcNode>();
            return empty_vector;
        }
        GlcNode currentNode = best;
        Vector<GlcNode> path = new Vector<GlcNode>();
        while (!(currentNode.parent == null)) {
            // this was a copy but i don't think it's necessary here
            path.add(currentNode);
            currentNode = currentNode.parent;
        }
        path.add(currentNode);
        if (forward) {
            Collections.reverse(path);
        }
        return path;
    }

    /**
     * Constructs the trajectory from the root to the leaf node in the goal
     * 
     * @param path is the sequence of nodes from root to leaf connected by edge
     *             relations
     * @returns a pointer to a trajectory object representing the solution
     *          trajectory
     */
    public InterpolatingPolynomial recoverTraj(final Vector<GlcNode> path) {
        if (path.size() < 2) {
            return null;
        }
        InterpolatingPolynomial opt_sol = path.get(1).trajectory_from_parent;
        for (int i = 2; i < path.size(); i++) {
            opt_sol.concatenate(path.get(i).trajectory_from_parent);
        }
        return opt_sol;
    }

    /**
     * The core iteration of the algorithm that pops the top of the queue and
     * forward integrates the dynamics with each of the controls
     */
    void expand() {
        // Increment the iteration count
        iter++;
        /////////////////
        //System.out.println("Planner iteration: " + iter);

        // If the queue is empty then the problem is not feasible at the current
        // resolution
        if (queue.isEmpty()) {
            System.out.println("---The queue is empty. Resolution too low or no solution at all.---");
            live = false;
            return;
        }

        // System.out.println("OPEN " + queue.size());
        // Pop the top of the queue for expansion
        GlcNode current_node = queue.poll();
        // queue.pop();

        // Once a goal node is found we clear the queue and keep the lowest cost node in
        // the goal
        if (found_goal) {
            double goal_time = goal.inGoal(current_node.trajectory_from_parent);
            if (goal_time >= 0) {
                double cost = current_node.parent.cost +
                        cf.cost(current_node.trajectory_from_parent,
                                current_node.control_from_parent,
                                current_node.trajectory_from_parent.initialTime(),
                                goal_time);
                if (cost < best.cost) {
                    best = current_node;
                    // run_time = clock() - tstart;
                    run_time = System.currentTimeMillis() - tstart;
                    live = false;
                    System.out.println("\n\nFound goal at iter: " + iter);
                    System.out.println("     solution cost: " + best.cost);
                    System.out.println("      running time: " + (float) run_time / (float) CLOCKS_PER_SEC);
                    System.out.println("  Simulation count: " + dynamics.sim_counter);
                    System.out.println("  Collision checks: " + obs.collision_counter);
                    System.out.println("       Size of set: " + partition_labels.size());
                    System.out.println("     Size of queue: " + queue.size());
                }
            }
        }

        // Stop the algorithm if the search tree reaches the depth or iteration limit
        if (current_node.depth >= depth_limit || iter > params.max_iter) {
            System.out.println("---exceeded depth or iteration limit---");
            live = false;
            return;
        }

        // A set of equivalence classes visited by new nodes made by expand
        Set<GlcStateEquivalenceClass> domains_needing_update = new TreeSet<GlcStateEquivalenceClass>();

        // Expand top of queue and store arcs in set of domains
        for (int i = 0; i < controls.size(); i++) {
            ///////////////////
            // System.out.println("CONTROL " + i);
            double[] c0;
            // Create a control signal spline which is a first order hold.
            // u(t)=c0+c1*t. If expanding root, just use u(t)=constant;
            if (current_node.parent == null) {
                // System.out.println("NO PARENT");
                c0 = controls.get(i);
            } else {
                // System.out.println("HAS PARENT");
                c0 = controls.get(current_node.u_idx);
            }
            /////////////////
            // System.out.println("c0 " + Arrays.toString(c0));
            double[] c1 = new double[c0.length];
            for (int j = 0; j < c0.length; ++j) {
                c1[j] = (controls.get(i)[j] - c0[j]) / expand_time;
            }
            Vector<double[]> segment = new Vector<double[]>();
            segment.add(c0);
            segment.add(c1);
            Vector<Vector<double[]>> linear_interp = new Vector<Vector<double[]>>();
            linear_interp.add(segment);
            // The above parameters are used to construct new_control
            InterpolatingPolynomial new_control = new InterpolatingPolynomial(linear_interp, expand_time,
                    current_node.time, controls.get(i).length, 2);
            /////
            // System.out.println("\nCONTROL");
            // new_control.printData();

            // Forward simulate with new_control to get a cubic spline between collocation
            // points
            InterpolatingPolynomial new_traj = dynamics.sim(current_node.time, current_node.time + expand_time,
                    current_node.state, new_control);
            GlcNode new_arc = new GlcNode(controls.size(),
                    i,
                    cf.cost(new_traj, new_control, current_node.time, current_node.time + expand_time)
                            + current_node.cost,
                    h.costToGo(new_traj.at(current_node.time + expand_time)),
                    new_traj.at(current_node.time + expand_time),
                    current_node.time + expand_time,
                    current_node,
                    new_traj,
                    new_control);

            // System.out.println("\nTRAJ");
            // new_traj.printData();

            // Create a region for the new trajectory
            double[] w = new double[new_arc.state.length];
            for (int j = 0; j < new_arc.state.length; ++j) {
                w[j] = inverse_cubicle_side_length * new_arc.state[j];
            }
            GlcStateEquivalenceClass d_new = new GlcStateEquivalenceClass(GlcMath.vecFloor(w));

            // Get the domain for the coordinate or create it and insert into labels.
            if (partition_labels.containsKey(d_new)) {
                // System.out.println("USE");
                d_new = partition_labels.get(d_new);
            } else {
                // System.out.println("PUT");
                partition_labels.put(d_new, d_new);
            }
            GlcStateEquivalenceClass bucket = d_new;
            // Add to a queue of domains that need inspection
            domains_needing_update.add(bucket);

            if (compare.compare(new_arc, bucket.label) < 0) {

                bucket.candidates.add(new_arc);
            } else {
                // System.out.println("NO BUCKET");
            }
        }

        //////////////
        // System.out.println("update set size " + domains_needing_update.size());

        // Go through the new trajectories and see if there is a possibility for
        // relabeling before collision check
        for (var open_domain : domains_needing_update) {
            // System.out.println("OPEN CANDIDATES " + open_domain.candidates.size());
            GlcStateEquivalenceClass current_domain = open_domain;
            // We go through the queue of candidates for relabeling/pushing in each set
            boolean found_best = false;
            while ((!found_best) && (!current_domain.candidates.isEmpty())) {
                // If the top of the candidate queue is cheaper than the label we should coll
                // check it
                if (compare.compare(current_domain.candidates.peek(), current_domain.label) < 0) {
                    GlcNode best_relabel_candidate = current_domain.candidates.peek();
                    InterpolatingPolynomial candidate_traj = best_relabel_candidate.trajectory_from_parent;// traj_from_parent[best_relabel_candidate];
                    if (obs.collisionFree(candidate_traj)) {
                        // Flag vertex if it's in the goal
                        double time = goal.inGoal(candidate_traj);
                        if (time >= 0) {
                            found_goal = true;
                        }
                        queue.add(best_relabel_candidate);// anything collision free at this point goes to queue
                        // why do we only choose the first best instead of the best from the whole set?
                       // if (!found_best) {
                            found_best = true;
                            current_domain.label = best_relabel_candidate;
                       // }
                    } else {
                        //System.out.println(" collision");
                        // candidate_traj.printData();
                    }
                } else {
                    //System.out.println(" more expensive");
                }
                current_domain.candidates.poll();
            }
            if (current_domain.empty()) {
                partition_labels.remove(current_domain);
            }
        }
        return;
    }

    /**
     * Method that calls expand until one of several flags is set for the algorithm
     * to stop
     */
    void expandWhileLive() {
        while (live) {
            /////////////////
            // System.out.println("EXPAND");

            expand();
        }
        return;
    }

    /**
     * Overload of plan() which provides some output on the result
     * 
     * @returns a struct containing info on whether a solution was found and what
     *          its cost is
     */
    public PlannerOutput plan() {
        expandWhileLive();
        PlannerOutput out = new PlannerOutput();
        out.cost = best.cost;
        out.time = (float) run_time / (float) CLOCKS_PER_SEC;
        out.solution_found = found_goal;// TO?DO change to found_goal
        return out;
    }

    /**
     * 
     * //
     */
    // boolean getSolution(InterpolatingPolynomial traj_out);

};
