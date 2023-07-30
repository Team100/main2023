package org.team100.glclib.glc_interface;

import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;

/**
 * This class defines the cost of a control signal together with the associated
 * trajectory for a particular problem instance
 *
 * The running cost of the cost functional must be globally Lipschitz continuous
 * on the feasible reason of the state space and the feasible set of control
 * inputs. A Lipschitz constant must be provided and the closer it is to the
 * smallest Lipschitz constant for the cost function, the better the performance
 * will be.
 */
public abstract class CostFunction {
    protected final double lipschitz_constant;

    /**
     * The constructor sets the Lipschitz constant for the running cost g in
     * c(x,u) = integral_t_0^tf g(x(t),u(t)) dt
     */
    public CostFunction(double _lipschitz_constant) {
        lipschitz_constant = _lipschitz_constant;
    }

    /**
     * The user must implement a running cost that integrates the cost along
     * a trajectory with a given input
     */
    public abstract double cost(final InterpolatingPolynomial trajectory_,
            final InterpolatingPolynomial control_, double t0_, double tf_);

    /**
     * This method returns the Lipschitz constant of the cost function
     * 
     * @returns The Lipschitz constant of the running cost in the cost function
     */
    double getLipschitzConstant() {
        return lipschitz_constant;
    }
};