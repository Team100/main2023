package org.team100.glclib;

import org.team100.glclib.glc_interface.CostFunction;
import org.team100.glclib.glc_interface.GoalRegion;
import org.team100.glclib.glc_interface.Heuristic;
import org.team100.glclib.glc_interface.Inputs;
import org.team100.glclib.glc_interface.Obstacles;
import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;
import org.team100.glclib.glc_numerical_integration.RungeKuttaTwo;

public class SampleInterfaces {

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
        double goal_radius, goal_radius_sqr;
        double[] error;
        double[] x_g;
        int resolution;

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
            double time = traj . initialTime();

            double dt = (traj . numberOfIntervals() * traj . intervalLength()) / resolution;
            for (int i = 0; i < resolution; i++) {
                time += dt;// don't need to check t0 since it was part of last traj
                for (int j = 0; j < x_g.length; ++j) {
                    error[j] = x_g[j] - traj . at(time)[j];
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
        public void flow(double[] dx, final double[] x, final double[] u) {
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
        double sample_resolution;

        public ArcLength(int _sample_resolution) {
            super(0.0);
            sample_resolution = _sample_resolution;
        }

        public double cost(final InterpolatingPolynomial traj,
                final InterpolatingPolynomial control, double t0, double tf) {
            double c = 0;
            double t = traj . initialTime();
            double dt = (tf - t0) / sample_resolution;
            for (int i = 0; i < sample_resolution; i++) {
                double[] traj_at_t = traj . at(t);
                double[] traj_at_t_dt = traj . at(t + dt);
                double [] diff = new double[traj_at_t.length];
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
        public         CarControlInputs(int num_steering_angles) {
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
        double radius_sqr;
        double[] center;
        int resolution;

        public SphericalGoal2(double _goal_radius_sqr,
                double[] _goal_center,
                int _resolution) {

            resolution = _resolution;
            center = _goal_center;
            radius_sqr = _goal_radius_sqr;
        }

        // Returns true if traj intersects goal and sets t to the first time at which
        // the trajectory is in the goal
        @Override
        public  double inGoal(final InterpolatingPolynomial traj) {
           double  time = traj . initialTime();

            double dt = (traj . numberOfIntervals() * traj . intervalLength()) / resolution;
            for (int i = 0; i < resolution; i++) {
                time += dt;// don't need to check t0 since it was part of last traj
                double[] state = traj . at(time);
                if (GlcMath.sqr(state[0] - center[0]) + GlcMath.sqr(state[1] - center[1]) < radius_sqr) {
                    return time;
                }
            }
            return -1;
        }
    };

    ////////////////////////////////////////////////////////
    //////// Problem Specific Admissible Heuristic///////////
    ////////////////////////////////////////////////////////
    public static class EuclideanHeuristic extends Heuristic {
        double radius;
        double[] goal;

        public EuclideanHeuristic(double[] _goal, double _radius) {
            radius = _radius;
            goal = _goal;
        }

        public  double costToGo(final double[] state) {
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
        public    void flow(double[] dx, final double[] x, final double[] u) {
            dx[0] = u[0] * Math.cos(x[2]);
            dx[1] = u[0] * Math.sin(x[2]);
            dx[2] = u[1];
        }

        public     double getLipschitzConstant() {
            return lipschitz_constant;
        }
    };

    ////////////////////////////////////////////////////////
    ///////////////// State Constraints//////////////////////
    ////////////////////////////////////////////////////////
    public static class PlanarDemoObstacles extends Obstacles {
        int resolution;
        double[] center1;
        double[] center2;

        public PlanarDemoObstacles(int _resolution) {
            resolution = _resolution;
            center1 = new double[] { 3.0, 2.0 };
            center2 = new double[] { 6.0, 8.0 };
        }

        @Override
        public  boolean collisionFree(final InterpolatingPolynomial traj) {
            double t = traj . initialTime();
            double dt = (traj . numberOfIntervals() * traj . intervalLength()) / resolution;
            double[] state;
            for (int i = 0; i < resolution; i++) {
                t += dt;// don't need to check t0 since it was part of last traj
                state = traj . at(t);

                // Disk shaped obstacles
                if (GlcMath.sqr(state[0] - center1[0]) + GlcMath.sqr(state[1] - center1[1]) <= 4.0 ||
                        GlcMath.sqr(state[0] - center2[0]) + GlcMath.sqr(state[1] - center2[1]) <= 4.0) {
                    return false;
                }
            }
            return true;
        }
    };

    /**
     * Since the pendulum has non-trivial dynamics, we will use the
     * zero heuristic for this example
     */
    public static class ZeroHeuristic extends Heuristic {
        public ZeroHeuristic() {
        }

        public  double costToGo(final double[] state) {
            return 0.0;
        }
    };

    public static class PendulumTorque extends Inputs {
        public PendulumTorque(int resolution) {
            double[] u = new double[1];
            for (double torque = -0.2; torque <= 0.2; torque += 0.4 / resolution) {
                u[0] = torque;
                addInputSample(u);
            }
        }
    };

    /**
     * The dynamic model for this example is a pendulum with
     * origin defined at the stable equilibrium. The input
     * is a torque applied at the pivot.
     */
    public static class InvertedPendulum extends RungeKuttaTwo {
        public // For the chosen coordinate system for the dynamic model, the Lipschitz
               // constant is 1.0
        InvertedPendulum(final double max_time_step_) {
            super(1.0, max_time_step_, 2);
        }

        @Override
        public    void flow(double[] dx, final double[] x, final double[] u) {
            dx[0] = x[1];
            dx[1] = u[0] - Math.sin(x[0]);
        }

        public double getLipschitzConstant() {
            return lipschitz_constant;
        }
    };

    /**
     * We will use the minimum time performance objective.
     */
    public static class MinTime extends CostFunction {
        double sample_resolution;

        public MinTime(int _sample_resolution) {

            super(0.0);
            sample_resolution = _sample_resolution;
        }

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
    public static class UnconstrainedSpace extends Obstacles {
        public UnconstrainedSpace(int _resolution) {
        }

        @Override
        public boolean collisionFree(final InterpolatingPolynomial traj) {
            return true;
        }

    }

}
