package org.team100.glclib.glc_numerical_integration;

import java.util.Arrays;
import java.util.Vector;

import org.team100.glclib.glc_interface.DynamicalSystem;
import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;

/**
 * This class implements the sim method of the base class Dynamical
 * System with a numerical integration scheme
 * 
 * Note that the user must still provide a concrete dynamical model with flow
 * implemented
 */
public abstract class RungeKuttaTwo extends DynamicalSystem {
    // These are temporary variables used in the Runge-Kutta 2 integration
    // method

    private final double[] x1;
    private final double[] x2;
    private final double[] f0;
    private final double[] f1;
    private final double[] f2;

    // This is the maximum time step that sim is allowed to use
    private final double max_time_step;

    // This is a temporary variable used by the integration scheme
    private double h;

    /**
     * The constructor sets the member parameters of this and the base class
     */
    public RungeKuttaTwo(
            double lipschitz_constant_,
            double max_time_step_,
            int state_dim_) {
        super(lipschitz_constant_);
        max_time_step = max_time_step_;
        x1 = new double[state_dim_];
        x2 = new double[state_dim_];
        f0 = new double[state_dim_];
        f1 = new double[state_dim_];
        f2 = new double[state_dim_];
    }

    /**
     * This method numerically integrates the dynamics
     * 
     * @param t0 is the initial time for the simulation
     * @param tf is the final time for the simulation
     * @param x0 is the initial state for the simulation
     * @param u  is the control input defined over the interval [t0,tf]
     * @returns the trajectory satisfying the dynamic equations
     */
    @Override
    public InterpolatingPolynomial sim(final double t0, final double tf, final double[] x0,
            final InterpolatingPolynomial u) {
        if (tf <= t0)
            throw new IllegalArgumentException();

        int num_steps = (int) Math.ceil((tf - t0) / max_time_step);
        double integration_step = (tf - t0) / num_steps;
        InterpolatingPolynomial solution = new InterpolatingPolynomial(integration_step, t0, x0.length, 4);
        solution.reserve(num_steps);
        // set initial state and time
        double[] state = x0;
        double time = t0;
 
        // integrate
        for (int i = 0; i < num_steps; i++) {
            // Use numerical integration scheme to compute a spline extending from state
            // with input u([t,t+integration_step])
            InterpolatingPolynomial traj_segment = step(state, u, time, time + integration_step);
            // add traj_segment to solution
            solution.concatenate(traj_segment);
            time += integration_step;
            state = traj_segment.at(time);
        }
        sim_counter++;
        return solution;
    }

    /**
     * This method implements one step of the Runge-Kutta 2 numerical integration
     * method
     * 
     * @param x0 the initial state for the integration step
     * @param u  the control input defined over the interval [t0,tf]
     * @param t1 the initial time for the integration step
     * @param t2 the final time for the integration step
     * @return segment is a cubic approximation of the trajectory from t1 to t2
     */
    InterpolatingPolynomial step(
            final double[] x0,
            final InterpolatingPolynomial u,
            final double t1,
            final double t2) {
        if (t1 >= t2)
            throw new IllegalArgumentException("[ERROR]: Integration step must be positive in RungeKuttaTwo");

        h = t2 - t1;
        flow(f0, x0, u.at(t1));
        for (int i = 0; i < f0.length; ++i) {
            x1[i] = x0[i] + 0.5 * h * f0[i];
        }
        // x1=x0+0.5*h*f0;
        flow(f1, x1, u.at(t1 + 0.5 * h));
        for (int i = 0; i < f1.length; ++i) {
            x2[i] = x0[i] + h * f1[i];
        }
        // x2=x0+h*f1;
        flow(f2, x2, u.at(t2));

        // Cubic interpolation between x0 and x2 with x'(t1)=f(x0,u(t0)) and
        // x'(t2)=f(x2,u(t2))
        Vector<double[]> cubic = new Vector<double[]>();
        // copy here because original copied them
        cubic.add(Arrays.copyOf(x0, x0.length));// t^0 term
        cubic.add(Arrays.copyOf(f0, f0.length));// t^1 term
        double[] e = new double[f0.length];
        for (int i = 0; i < f0.length; ++i) {
            e[i] = (-2.0 * f0[i] + 3.0 * f1[i] - f2[i]) / h;
        }
        cubic.add(e);// t^2 term
        e = new double[f0.length];
        for (int i = 0; i < f0.length; ++i) {
            e[i] = (f0[i] - 2.0 * f1[i] + f2[i]) / (h * h);
        }
        cubic.add(e);// t^3 term
        Vector<Vector<double[]>> knot_point = new Vector<Vector<double[]>>();
        knot_point.add(cubic);
        return new InterpolatingPolynomial(knot_point, t2 - t1, t1, x0.length, 4);
    }
}