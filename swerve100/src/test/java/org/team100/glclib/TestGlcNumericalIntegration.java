package org.team100.glclib;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Vector;

import org.junit.jupiter.api.Test;
import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;
import org.team100.glclib.glc_numerical_integration.RungeKuttaTwo;

public class TestGlcNumericalIntegration {

    // A concrete implementation of a dynamical system is required to test the
    // numerical integration methods
    class SingleIntegrator extends RungeKuttaTwo {
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

    // This tests that the numerical integration integrates the input correctly for
    // the given dynamical system
    @Test
    public void testOrder() {
        // Create an instance of a 2D single integrator
        SingleIntegrator holonomic_model = new SingleIntegrator(1.0);
        // Construct an InerpolatingPolynomial for a constant control input of (1,0) for
        // 1 time unit
        double[] initial_state = new double[] { 0.0, 0.0 };
        double[] control = new double[] { 1.0, 0.0 };
        Vector<double[]> zero_order_interp = new Vector<double[]>();
        zero_order_interp.add(control);
        Vector<Vector<double[]>> zero_order_interp_array = new Vector<Vector<double[]>>();
        zero_order_interp_array.add(zero_order_interp);
        InterpolatingPolynomial control_segment = new InterpolatingPolynomial(zero_order_interp_array, 1.0, 0.0, 2, 1);
        // Simulate dynamics with input control_segment
        InterpolatingPolynomial traj_segment = holonomic_model.sim(0.0, 1.0, initial_state, control_segment);

        // Check that the resulting trajectory matches hand-calculated solution of x(t)
        // = (t,0)
        for (double t = 0; t <= 1.0; t += 0.1) {
            assertEquals(traj_segment.at(t)[0], t, 1e-4);
            assertEquals(traj_segment.at(t)[1], 0.0, 1e-4);
        }
    }

}
