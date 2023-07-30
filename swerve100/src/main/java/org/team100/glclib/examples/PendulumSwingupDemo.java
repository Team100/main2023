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

public class PendulumSwingupDemo {
    /**
     * This is an example of a discrete input set for the torque limited
     * pendulum swingup problem where the sampling of inputs from the
     * addmissible torque inputs is equal to the resolution parameter for
     * the glc algorithm.
     */
    public static class PendulumTorque extends Inputs {
        public PendulumTorque(int resolution) {

            double[] u = new double[1];
            double max_torque = 0.1;
            for (double torque = -max_torque; torque <= max_torque; torque += 2.0 * max_torque / resolution) {
                u[0] = torque;
                addInputSample(u);
            }
        }
    };

    /**
     * The goal region for this example is a small circular domain in
     * the angle-velocity space for the pendulum centered where
     * the pendulum is in the inverted position and the velocity is zero.
     * It is required that the goal region have a nonempty interior.
     */
    public static class SphericalGoal extends GoalRegion {
        private          double goal_radius;
        private   double goal_radius_sqr;
        private final  double[] error;
        private  double[] x_g;
        private final         int num_samples;

        public SphericalGoal(final int _state_dim,
                final double _goal_radius,
                int _num_samples) {
            x_g = new double[_state_dim];
            num_samples = _num_samples;
            goal_radius = _goal_radius;
            error = new double[_state_dim];
            goal_radius_sqr = GlcMath.sqr(goal_radius);
        }

        @Override
        public double inGoal(final InterpolatingPolynomial traj) {
            double time = traj.initialTime();
            double dt = (traj.numberOfIntervals() * traj.intervalLength()) / num_samples;
            for (int i = 0; i < num_samples; i++) {
                time += dt;
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

    /**
     * Since the pendulum has non-trivial dynamics, we will use the
     * zero heuristic for this example
     */
    public static class ZeroHeuristic extends Heuristic {
        public ZeroHeuristic() {
        }

        @Override
        public double costToGo(final double[] state) {
            return 0.0;
        }
    };

    /**
     * The dynamic model for this example is a pendulum with
     * origin defined at the stable equilibrium. The input
     * is a torque applied at the pivot.
     */
    public static class InvertedPendulum extends RungeKuttaTwo {
        // For the chosen coordinate system for the dynamic model, the Lipschitz
        // constant is 1.0
        public InvertedPendulum(final double max_time_step_) {
            super(1.0, max_time_step_, 2);
        }

        @Override
        public void flow(final double[] dx, final double[] x, final double[] u) {
            dx[0] = x[1];
            dx[1] = u[0] - Math.sin(x[0]);
        }

        @Override
        public double getLipschitzConstant() {
            return lipschitz_constant;
        }
    };

    /**
     * We will use the minimum time performance objective.
     */
    public static class MinTime extends CostFunction {
       // private final     double sample_resolution;

        public MinTime(int _sample_resolution) {
            super(0.0);
          //  sample_resolution = _sample_resolution;
        }

        @Override
        public double cost(final InterpolatingPolynomial traj,
                final InterpolatingPolynomial control,
                double t0,
                double tf) {
            return tf - t0;
        }
    };

    /**
     * This is an unconstrained free space all states are collision free
     */
    public static class PendulumStateSpace extends Obstacles {
        public PendulumStateSpace(int _resolution) {
        }

        @Override
        public boolean collisionFree(final InterpolatingPolynomial traj) {
            if (Math.abs(traj.at(traj.initialTime())[0]) > 3.2

                    || Math.abs(traj.at(traj.initialTime())[1]) > 3.2) {
                return false;
            }
            return true;
        }
    };

    public static void main(String... args) {

        // Motion planning algorithm parameters
        GlcParameters alg_params = new GlcParameters();
        alg_params.res = 5;
        alg_params.control_dim = 1;
        alg_params.state_dim = 2;
        alg_params.depth_scale = 100;
        alg_params.dt_max = 5.0;
        alg_params.max_iter = 50000;
        alg_params.time_scale = 7;
        alg_params.partition_scale = 2.5;
        alg_params.x0 = new double[] { 0.0, 0.0 };

        // Create a dynamic model
        InvertedPendulum dynamic_model = new InvertedPendulum(alg_params.dt_max);

        // Create the control inputs
        PendulumTorque controls = new PendulumTorque(alg_params.res);

        // Create the cost function
        MinTime performance_objective = new MinTime(4);

        // Create instance of goal region
        double[] xg = new double[] { Math.PI, 0.0 };

        SphericalGoal goal = new SphericalGoal(xg.length, 0.25, 6);
        goal.setGoal(xg);

        // Create the obstacles.
        PendulumStateSpace obstacles = new PendulumStateSpace(4);

        // Create a heuristic for the current goal
        ZeroHeuristic heuristic = new ZeroHeuristic();

        // Construct the planner
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
            solution.printData();
            GlcLogging.trajectoryToFile("pendulum_swingup_demo.txt", "./", solution, 500);
        }
        GlcLogging.nodesToFile("pendulum_swingup_demo_nodes.txt", "./", planner.partition_labels.keySet());
    }
}