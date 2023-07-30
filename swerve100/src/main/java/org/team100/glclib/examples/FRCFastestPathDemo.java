package org.team100.glclib.examples;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.Vector;

import org.team100.glclib.GlcLogging;
import org.team100.glclib.GlcMath;
import org.team100.glclib.GlcNode;
import org.team100.glclib.GlcParameters;
import org.team100.glclib.Planner;
import org.team100.glclib.PlannerOutput;
import org.team100.glclib.glc_interface.CostFunction;
import org.team100.glclib.glc_interface.DynamicalSystem;
import org.team100.glclib.glc_interface.GoalRegion;
import org.team100.glclib.glc_interface.Heuristic;
import org.team100.glclib.glc_interface.Inputs;
import org.team100.glclib.glc_interface.Obstacles;
import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;
import org.team100.glclib.glc_numerical_integration.RungeKuttaTwo;

/**
 * FRC path planner using 2023 game map.
 */
class FRCFastestPathDemo {

    /**
     * Discretization of Control Inputs
     * 
     * A finite subset of admissible control inputs parameterized by a resolution so
     * that this finite set converges to a dense subset with increasing resolution.
     */
    public static class ControlInputs2D extends Inputs {
        private static final double kMaxForce = 1;

        /** always max force */
        public ControlInputs2D(int num_inputs) {
            // always ok to do nothing
            addInputSample(new double[]{0.0, 0.0});
            double angle_increment = 2.0 * Math.PI / num_inputs;
            for (double angle = 0; angle < 2.0 * Math.PI; angle += angle_increment) {
                double[] u = new double[2];
                u[0] = kMaxForce * Math.sin(angle);
                u[1] = kMaxForce * Math.cos(angle);
                addInputSample(u);
            }
        }
    };

    /**
     * Discretization of Control Inputs
     * 
     * A finite subset of admissible control inputs parameterized by a resolution so
     * that this finite set converges to a dense subset with increasing resolution.
     */
    public static class MultiForceControlInputs2D extends Inputs {
        private static final double kMaxForce = 1;

        /** more than one force level */
        public MultiForceControlInputs2D(int num_inputs) {
            int speeds = 3;
            int angles = num_inputs / speeds;
            double force_increment = kMaxForce / speeds;
            double angle_increment = 2.0 * Math.PI / angles;
            for (double force = kMaxForce; force > 0; force -= force_increment) {
                for (double angle = 0; angle < 2.0 * Math.PI; angle += angle_increment) {
                    double[] u = new double[2];
                    u[0] = force * Math.sin(angle);
                    u[1] = force * Math.cos(angle);
                    addInputSample(u);
                }
            }
        }
    };

    /**
     * Goal Checking Interface
     * 
     * A goal checking subroutine that determines
     * if a trajectory object intersects the goal set.
     */
    public static class SphericalGoal extends GoalRegion {
        private final double goal_radius;
        private final double goal_radius_sqr;
        private final double[] error;
        private final double[] x_g;
        private final int resolution;

        public SphericalGoal(final double[] _x_g, final double _goal_radius, int _resolution) {
            if (_x_g.length != 4)
                throw new IllegalArgumentException();
            x_g = _x_g;
            goal_radius = _goal_radius;
            resolution = _resolution;
            error = new double[_x_g.length];
            goal_radius_sqr = GlcMath.sqr(goal_radius);
        }

        // Returns true if traj intersects goal and sets t to the first time at which
        // the trajectory is in the goal
        @Override
        public double inGoal(final InterpolatingPolynomial traj) {
            double time = traj.initialTime();
            double dt = (traj.numberOfIntervals() * traj.intervalLength()) / resolution;
            for (int i = 0; i < resolution; i++) {
                time += dt;
                // don't need to check t0 since it was part of last traj
                for (int j = 0; j < x_g.length; ++j) {
                    error[j] = x_g[j] - traj.at(time)[j];
                }
                if (GlcMath.dot(error, error) < goal_radius_sqr) {
                    return time;
                }
            }
            return -1;
        }
    };

    /**
     * Problem Specific Admissible Heuristic
     * 
     * An admissible heuristic that underestimates
     * the optimal cost-to-go from every feasible state.
     * One can always use h(x)=0 for all x as a heuristic.
     */
    public static class EuclideanHeuristic extends Heuristic {
        private final double radius;
        private final double[] goal;
        private final double max_speed;

        public EuclideanHeuristic(double[] _goal, double _radius, double max_speed) {
            if (_goal.length != 4)
                throw new IllegalArgumentException();
            radius = _radius;
            goal = _goal;
            this.max_speed = max_speed;
        }

        @Override
        public double costToGo(final double[] state) {
            if (state.length != 4)
                throw new IllegalArgumentException();

            // the goal is to underestimate the remaining cost, which is elapsed time (see
            // below).

            // zero works but then the search is not informed at all.
            // return 0;

            double gx = goal[0];
            double gy = goal[1];
            double gvx = goal[2];
            double gvy = goal[3];

            double px = state[0];
            double py = state[1];
            double vx = state[2];
            double vy = state[3];

            double distance = Math.max(0.0, Math.sqrt(GlcMath.sqr(gx - px) +
                    GlcMath.sqr(gy - py)) - radius);

            // the current speed might yield an overestimate of the remaining time...
            // double speed = Math.sqrt(GlcMath.sqr(gvx - vx) + GlcMath.sqr(gvy - vy));

            double eta = distance / max_speed;
            return eta;
        }
    };

    public static class ZeroHeuristic extends Heuristic {
        public ZeroHeuristic() {
        }

        @Override
        public double costToGo(final double[] state) {
            return 0.0;
        }
    };

    /**
     * State space model.
     * 
     * A dynamic model describing the response of the system to control inputs and
     * also a lipschitz constant for the model.
     * 
     * Two dimensions:
     * x[0] = x position
     * x[1] = y position
     * x[2] = x velocity
     * x[3] = y velocity
     */
    public static class SingleIntegrator extends RungeKuttaTwo {
        private static final double mass = 0.5;
        private final double max_speed ;

        SingleIntegrator(final double max_time_step_, double max_speed) {
            super(0.0, max_time_step_, 4);
            this.max_speed = max_speed;
        }

        /**
         * @param dx derivative of x
         * @param x  state, 0:x, 1:y, 2:vx, 3:vy
         * @param u  control, 0:ax, 1:ay
         */
        @Override
        public void flow(final double[] dx, final double[] x, final double[] u) {
            if (dx.length != 4)
                throw new IllegalArgumentException();
            if (x.length != 4)
                throw new IllegalArgumentException();
            if (u.length != 2)
                throw new IllegalArgumentException();
            // TODO add something about motor curves here
            // for now just do x and y separately
            double ax = u[0] / mass;
            double ay = u[1] / mass;
            double vx = x[2];
            double vy = x[3];
            double new_vx = vx + ax;
            double new_vy = vy + ay;
            double new_v_angle = Math.atan2(new_vy, new_vx);
            double new_v_mag = Math.min(max_speed, Math.hypot(new_vx, new_vy));
            new_vx = new_v_mag * Math.cos(new_v_angle);
            new_vy = new_v_mag * Math.sin(new_v_angle);
            ax = new_vx - vx;
            ay = new_vy - vy;

            dx[0] = new_vx;
            dx[1] = new_vy;
            dx[2] = ax;
            dx[3] = ay;

            // dx[0] = x[2] + u[0] / mass;
            // dx[1] = x[3] + u[1] / mass;
            // dx[2] = u[0] / mass;
            // dx[3] = u[1] / mass;
        }

        @Override
        public double getLipschitzConstant() {
            return 0.0;
        }
    };

    /**
     * Cost function: elapsed time.
     */
    public static class MinTime extends CostFunction {
        public MinTime() {
            super(0.0);
        }

        @Override
        public double cost(final InterpolatingPolynomial traj,
                final InterpolatingPolynomial control, double t0, double tf) {
            return tf - t0;
        }
    };

    /**
     * State Constraints
     * 
     * A feasibility or collision checking function.
     * 
     * TODO: moving obstacles
     */
    public static class PlanarDemoObstacles extends Obstacles {
        private static final double kDia = 0.5;
        private final int resolution;

        public PlanarDemoObstacles(int _resolution) {
            resolution = _resolution;
        }

        @Override
        public boolean collisionFree(final InterpolatingPolynomial traj) {
            double t = traj.initialTime();
            double dt = (traj.numberOfIntervals() * traj.intervalLength()) / resolution;
            double[] state;
            for (int i = 0; i < resolution; i++) {
                t += dt;
                // don't need to check t0 since it was part of last traj
                state = traj.at(t);
                double x = state[0];
                double y = state[1];
                if (x - kDia < 1.43 && y - kDia < 5.49) {
                    // System.out.println(" nodes");
                    ++collision_counter;
                    return false;
                }
                if (x + kDia > 13.18 && y - kDia < 5.49) {
                    // System.out.println(" opponent community");
                    ++collision_counter;
                    return false;
                }
                if (x - kDia < 3.36 && y + kDia > 5.49) {
                    // System.out.println(" opponent loading");
                    ++collision_counter;
                    return false;
                }
                if (x - kDia < 6.71 && y + kDia > 6.75) {
                    // System.out.println(" opponent loading");
                    ++collision_counter;
                    return false;
                }
                if (x + kDia > 16.54) {
                    // System.out.println(" far baseline " + (x + kDia));
                    ++collision_counter;
                    return false;
                }
                if (x - kDia < 0) {
                    // System.out.println(" near baseline");
                    ++collision_counter;
                    return false;
                }
                if (y + kDia > 8.02) {
                    // System.out.println(" sideline");
                    ++collision_counter;
                    return false;
                }
                if (y - kDia < 0) {
                    // System.out.println(" sideline");
                    ++collision_counter;
                    return false;
                }
                if (x + kDia > 2.98
                        && x - kDia < 4.91
                        && y + kDia > 1.51
                        && y - kDia < 3.98) {
                    // System.out.println(" red charge station");
                    ++collision_counter;
                    return false;
                }
                if (x + kDia > 11.63
                        && x - kDia < 13.56
                        && y + kDia > 1.51
                        && y - kDia < 3.98) {
                    // System.out.println(" blue charge station");
                    ++collision_counter;
                    return false;
                }
                // moving opponents
                double opp1x = 4 + 2 * t;
                double oppSize = 1;
                double opp1y = 6 - t;
                if (x + kDia >  opp1x-oppSize/2
                        && x - kDia <  opp1x+oppSize/2
                        && y + kDia >  opp1y-oppSize/2
                        && y - kDia <  opp1y+oppSize/2) {
                    // System.out.println(" opponent 1");
                    ++collision_counter;
                    return false;
                }
                double opp2x = 4 + 2 * t;
                double opp2y = 7 - t/4;
                if (x + kDia > opp2x-oppSize/2
                        && x - kDia <  opp2x+oppSize/2
                        && y + kDia >  opp2y-oppSize/2
                        && y - kDia <  opp2y+oppSize/2) {
                    // System.out.println(" opponent 2");
                    ++collision_counter;
                    return false;
                }
                // defender
                double opp3x = 8;
                double opp3y = 5 ;
                if (x + kDia > opp3x-oppSize/2
                        && x - kDia <  opp3x+oppSize/2
                        && y + kDia >  opp3y-oppSize/2
                        && y - kDia <  opp3y+oppSize/2) {
                    // System.out.println(" opponent 2");
                    ++collision_counter;
                    return false;
                }
            }
            return true;
        }
    };

    public static void main(String... args) {
        GlcParameters alg_params = new GlcParameters();
        alg_params.res = 5;
        alg_params.control_dim = 2;
        alg_params.state_dim = 4;
        alg_params.depth_scale = 10;
        alg_params.dt_max = 5.0;
        alg_params.max_iter = 500000;
        alg_params.time_scale = 2;
        alg_params.partition_scale = 6;
        // starting point is at the substation (x, y, vx, vy)
        alg_params.x0 = new double[] { 15.5, 6.750, 0, 0 };

        double max_speed = 5;
        DynamicalSystem dynamic_model = new SingleIntegrator(alg_params.dt_max, max_speed);
        Inputs controls = new ControlInputs2D(alg_params.res);
        CostFunction performance_objective = new MinTime();
        // Goal is 10 cm radius just outside the center scoring tag, stopped.
        double[] xg = new double[] { 1.93, 2.748, 0, 0 }; // (x, y, vx, vy)
        double goal_radius = 0.5; // make it bigger than the partition (used to be 0.2)
        int goal_resolution = 1; // just check the endpoint (used to be 4)
        GoalRegion goal = new SphericalGoal(xg, goal_radius, goal_resolution);
        int obstacle_resolution = 4; // if this is too low then edges overlap obstacles
        Obstacles obstacles = new PlanarDemoObstacles(obstacle_resolution);
        Heuristic heuristic = new EuclideanHeuristic(xg, goal_radius, max_speed);
        //Heuristic heuristic = new ZeroHeuristic();
        Planner planner = new Planner(obstacles,
                goal,
                dynamic_model,
                heuristic,
                performance_objective,
                alg_params,
                controls.readInputs());

        PlannerOutput out = planner.plan();
        if (out.solution_found) {

            Vector<GlcNode> path = planner.pathToRoot(true);
            try {
                PrintWriter pathWriter = new PrintWriter("path.txt");

                for (int i = 0; i < path.size(); ++i) {
                    double time = path.get(i).time;
                    double[] state = path.get(i).state;
                    double[] u = controls.readInputs().get(path.get(i).u_idx);
                    pathWriter.printf("%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n", 
                    time, state[0], state[1], state[2], state[3], u[0], u[1]);
                }
                pathWriter.close();
            } catch (IOException e) {

            }

            InterpolatingPolynomial solution = planner.recoverTraj(path);
            if (solution != null) {
                solution.printSpline(20, "Solution");
                GlcLogging.trajectoryToFile("frc_shortest_path_demo.txt", "./", solution, 50);
            }
            GlcLogging.nodesToFile("frc_shortest_path_demo_nodes.txt", "./", planner.partition_labels.keySet());
        }
    }

}