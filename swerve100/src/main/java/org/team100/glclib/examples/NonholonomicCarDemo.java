package org.team100.glclib.examples;

import java.util.Vector;

import org.team100.glclib.GlcLogging;
import org.team100.glclib.GlcMath;
import org.team100.glclib.GlcNode;
import org.team100.glclib.GlcParameters;
import org.team100.glclib.Planner;
import org.team100.glclib.PlannerOutput;
import org.team100.glclib.glc_interface.CostFunction;
import org.team100.glclib.glc_interface.GoalRegion;
import org.team100.glclib.glc_interface.Heuristic;
import org.team100.glclib.glc_interface.Inputs;
import org.team100.glclib.glc_interface.Obstacles;
import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;
import org.team100.glclib.glc_numerical_integration.RungeKuttaTwo;

public class NonholonomicCarDemo {

    ////////////////////////////////////////////////////////
    ///////// Discretization of Control Inputs///////////////
    ////////////////////////////////////////////////////////
    public static class ControlInputs2D extends Inputs {
        // uniformly spaced points on a circle
        public ControlInputs2D(int num_inputs) {

            double[] u = new double[2];
            for (int i = 0; i < num_inputs; i++) {
                u[0] = Math.sin(2.0 * i * Math.PI / num_inputs);
                u[1] = Math.cos(2.0 * i * Math.PI / num_inputs);
                addInputSample(u);
            }
        }
    };

    ////////////////////////////////////////////////////////
    /////////////// Goal Checking Interface//////////////////
    ////////////////////////////////////////////////////////
    public static class SphericalGoal extends GoalRegion {
        private  double goal_radius;
        private  double goal_radius_sqr;
        private final  double[] error;
        private    double[] x_g;
        private final  int resolution;

        public SphericalGoal(final int _state_dim,
                final double _goal_radius,
                int _resolution) {

            x_g = new double[_state_dim];
            resolution = _resolution;
            goal_radius = _goal_radius;
            error = new double[_state_dim];

            goal_radius_sqr = GlcMath.sqr(goal_radius);
        }

        // Returns true if traj intersects goal and sets t to the first time at which
        // the trajectory is in the goal
        @Override
        public double inGoal(final InterpolatingPolynomial traj) {
            double time = traj.initialTime();

            double dt = (traj.numberOfIntervals() * traj.intervalLength()) / resolution;
            for (int i = 0; i < resolution; i++) {
                time += dt;// don't need to check t0 since it was part of last traj
                for (int j = 0; j < x_g.length; ++j) {
                    error[j] = x_g[j] - traj.at(time)[j];
                }

                if (GlcMath.dot(error, error) < goal_radius_sqr) {
                    return time;
                }
            }
            return -1;
        }

        void setRadius(double r) {
            goal_radius = r;
            goal_radius_sqr = r * r;
        }

        double getRadius() {
            return goal_radius;
        }

        void setGoal(double[] _x_g) {
            x_g = _x_g;
        }

        double[] getGoal() {
            return x_g;
        }

    };

    ////////////////////////////////////////////////////////
    ///////////////// Dynamic Model//////////////////////////
    ////////////////////////////////////////////////////////
    public static class SingleIntegrator extends RungeKuttaTwo {
        public SingleIntegrator(final double max_time_step_) {
            super(0.0, max_time_step_, 2);
        }

        @Override
        public void flow(final double[] dx, final double[] x, final double[] u) {
            for (int i = 0; i < u.length; ++i) {
                dx[i] = u[i];
            }
        }

        public double getLipschitzConstant() {
            return 0.0;
        }
    };

    ////////////////////////////////////////////////////////
    ///////////////// Cost Function//////////////////////////
    ////////////////////////////////////////////////////////
    public static class ArcLength extends CostFunction {
        private final  double sample_resolution;

        public ArcLength(int _sample_resolution) {
            super(0.0);
            sample_resolution = _sample_resolution;

        }

        @Override
        public double cost(final InterpolatingPolynomial traj,
                final InterpolatingPolynomial control,
                double t0,
                double tf) {

            double c = 0;
            double t = traj.initialTime();
            double dt = (tf - t0) / sample_resolution;
            for (int i = 0; i < sample_resolution; i++) {
                double[] traj_at_t_dt = traj.at(t + dt);
                double[] traj_at_t = traj.at(t);
                double[] diff = new double[traj_at_t.length];
                for (int j = 0; j < traj_at_t.length; ++j) {
                    diff[j] = traj_at_t_dt[j] - traj_at_t[j];
                }
                c += GlcMath.norm2(diff);
                t += dt;
            }
            return c;
        }
    };

    public static class CarControlInputs extends Inputs {

        // uniformly spaced points on a circle
        public CarControlInputs(int num_steering_angles) {

            // Make all pairs (forward_speed,steering_angle)
            double[] car_speeds = new double[] { 1.0 };// Pure path planning

            double[] steering_angles = GlcMath.linearSpace(-0.0625 * Math.PI, 0.0625 * Math.PI, num_steering_angles);

            double[] control_input = new double[2];
            for (double vel : car_speeds) {
                for (double ang : steering_angles) {
                    control_input[0] = vel;
                    control_input[1] = ang;
                    addInputSample(control_input);
                }
            }
        }

    };

    ////////////////////////////////////////////////////////
    /////////////// Goal Checking Interface//////////////////
    ////////////////////////////////////////////////////////
    public static class SphericalGoal2 extends GoalRegion {
        private final   double radius_sqr;
        private final  double[] center;
        private final  int resolution;

        public SphericalGoal2(double _goal_radius_sqr,
                double[] _goal_center,
                int _resolution) {
            resolution = _resolution;
            center = _goal_center;
            radius_sqr = _goal_radius_sqr;
        }

        // Returns true if traj intersects goal and sets t to the first time at which
        // the trajectory is in the goal
        public double inGoal(final InterpolatingPolynomial traj) {
            double time = traj.initialTime();

            double dt = (traj.numberOfIntervals() * traj.intervalLength()) / resolution;
            for (int i = 0; i < resolution; i++) {
                time += dt;// don't need to check t0 since it was part of last traj
                double[] state = traj.at(time);
                if (GlcMath.sqr(state[0] - center[0]) + GlcMath.sqr(state[1] - center[1]) < radius_sqr) {
                    return time;
                }
            }
            return -1;
        }

    }

    ////////////////////////////////////////////////////////
    //////// Problem Specific Admissible Heuristic///////////
    ////////////////////////////////////////////////////////
    public static class EuclideanHeuristic extends Heuristic {
        private final   double radius;
        private final  double[] goal;

        public EuclideanHeuristic(double[] _goal, double _radius) {
            radius = _radius;
            goal = _goal;
        }

        @Override
        public double costToGo(final double[] state) {
            return Math.max(0.0, Math.sqrt(GlcMath.sqr(goal[0] - state[0]) + GlcMath.sqr(goal[1] - state[1])) - radius);
            // offset by goal radius
        }
    };

    ////////////////////////////////////////////////////////
    ///////////////// Dynamic Model//////////////////////////
    ////////////////////////////////////////////////////////
    public static class CarNonholonomicConstraint extends RungeKuttaTwo {
        public CarNonholonomicConstraint(final double _max_time_step) {
            super(1.0, _max_time_step, 3);
        }

        @Override
        public void flow(final double[] dx, final double[] x, final double[] u) {
            dx[0] = u[0] * Math.cos(x[2]);
            dx[1] = u[0] * Math.sin(x[2]);
            dx[2] = u[1];
        }

        @Override
        public double getLipschitzConstant() {
            return lipschitz_constant;
        }
    };

    ////////////////////////////////////////////////////////
    ///////////////// State Constraints//////////////////////
    ////////////////////////////////////////////////////////
    public static class PlanarDemoObstacles extends Obstacles {
        private final  int resolution;
        private final  double[] center1;
        private final double[] center2;

        public PlanarDemoObstacles(int _resolution) {
            resolution = _resolution;
            center1 = new double[] { 3.0, 2.0 };
            center2 = new double[] { 6.0, 8.0 };
        }

        @Override
        public boolean collisionFree(final InterpolatingPolynomial traj) {
            double t = traj.initialTime();
            double dt = (traj.numberOfIntervals() * traj.intervalLength()) / resolution;
            double[] state;
            for (int i = 0; i < resolution; i++) {
                t += dt;// don't need to check t0 since it was part of last traj
                state = traj.at(t);

                // Disk shaped obstacles
                if (GlcMath.sqr(state[0] - center1[0]) + GlcMath.sqr(state[1] - center1[1]) <= 4.0 ||
                        GlcMath.sqr(state[0] - center2[0]) + GlcMath.sqr(state[1] - center2[1]) <= 4.0) {
                    return false;
                }
            }
            return true;
        }
    };

    ////////////////////////////////////////////////////////
    /////////////// Run a planning query in main/////////////
    ////////////////////////////////////////////////////////

    public static void main(String... args) {

        // Motion planning algorithm parameters
        GlcParameters alg_params = new GlcParameters();
        ;
        alg_params.res = 21;
        alg_params.control_dim = 2;
        alg_params.state_dim = 3;
        alg_params.depth_scale = 100;
        alg_params.dt_max = 5.0;
        alg_params.max_iter = 50000;
        alg_params.time_scale = 20;
        alg_params.partition_scale = 60;
        alg_params.x0 = new double[] { 0.0, 0.0, Math.PI / 2.0 };

        // Create a dynamic model
        CarNonholonomicConstraint dynamic_model = new CarNonholonomicConstraint(alg_params.dt_max);

        // Create the control inputs
        CarControlInputs controls = new CarControlInputs(alg_params.res);

        // Create the cost function
        ArcLength performance_objective = new ArcLength(4);

        // Create instance of goal region
        double goal_radius_sqr = .25;

        double[] goal_center = new double[] { 10.0, 10.0 };

        SphericalGoal2 goal = new SphericalGoal2(goal_radius_sqr, goal_center, 10);

        // Create the obstacles
        PlanarDemoObstacles obstacles = new PlanarDemoObstacles(10);

        // Create a heuristic for the current goal
        EuclideanHeuristic heuristic = new EuclideanHeuristic(goal_center, Math.sqrt(goal_radius_sqr));

        Planner planner = new Planner(obstacles,
                goal,
                dynamic_model,
                heuristic,
                performance_objective,
                alg_params,
                controls.readInputs());
        // Run the planner and print solution
        PlannerOutput out = planner.plan();
        if (out.solution_found) {
            Vector<GlcNode> path = planner.pathToRoot(true);
            InterpolatingPolynomial solution = planner.recoverTraj(path);
            solution.printSpline(20, "Solution");
            GlcLogging.trajectoryToFile("nonholonomic_car_demo.txt", "./", solution, 500);
            GlcLogging.nodesToFile("nonholonomic_car_demo_nodes.txt", "./", planner.partition_labels.keySet());
        }
    }
}