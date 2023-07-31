package org.team100.glclib.glc_interface;

import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;

/**
 *  The user must define a dynamical system whose differential constraints
 * must be satisfied by a solution to a particular problem
 */
public abstract class DynamicalSystem {

    // keeps track of how many calls to the simulation method
    // there are in a planning query
    public int sim_counter = 0;

    // the lipschitz constant of the dynamics (in the state variable!!!)
    protected final double lipschitz_constant;

    /**
     * The base constructor sets the lipschitz_constant for the dynamic model
     * 
     * @param lipschitz_constant_ is the value that lipchitz_constant is set to
     */
    public DynamicalSystem(double lipschitz_constant_) {
        lipschitz_constant = lipschitz_constant_;
    }

    /**
     * This method represents the function defining the differential
     * constraint x'(t)=f(x(t),u(t))
     * 
     * @param dx [out] the time derivative of the state at x given x and u
     * @param x  state of the dynamic system
     * @param u  control input to the dynamical system
     */
    public abstract void flow(final double[] dx, final double[] x, final double[] u);

    /**
     * @returns the Lipschitz constant for the dynamic system
     */
    public abstract double getLipschitzConstant();

    /**
     * This method integrates the differential equation over a given time
     * interval, with a given control signal, and with given initial state.
     * 
     * If a closed form solution to the dynamics is available, it should be used
     * here. The most general class of systems
     * where this is the case is linear systems. On the other hand, if numerical
     * integration is required see the
     * derived class RungeKuttaTwo which is provided with this library for derived
     * class implementing a numerical
     * integration scheme.
     * 
     * @param t0 is the initial time of the simulation
     * @param tf is the final time of the simulation
     * @param x0 is the initial state for the simulation
     * @param u  is the control signal for the simulation that must be defined
     *           over the interval [t0,tf]
     * @returns the trajectory satisfying the differential equation
     *          for the input data
     */
    public abstract InterpolatingPolynomial sim(double t0,
            double tf,
            final double[] x0,
            final InterpolatingPolynomial u);
};