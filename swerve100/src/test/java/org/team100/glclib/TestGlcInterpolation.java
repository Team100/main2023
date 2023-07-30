package org.team100.glclib;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Vector;

import org.junit.jupiter.api.Test;
import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;

public class TestGlcInterpolation {

    /**
     * This test checks that the constructor for builds the interpolating polynomial
     * correctly
     */
    @Test
    public void testConstructor() {

        // These are the test values for the InterpolatingPolynomial's attributes
        double initial_time = 5.0;
        double interval_length = 2.0;
        int degree = 2;
        int dimension = 3;
        int number_of_intervals = 3;

        // Here the coefficient array for an interpolating polynomial spline is created
        Vector<Vector<double[]>> coefficient_array = new Vector<Vector<double[]>>();
        for (int j = 0; j < number_of_intervals; j++) {
            // This will be the coefficients of a polynomial in R^3 with the basis c_i*t^i
            // for i ranging from 0 to 2
            Vector<double[]> polynomial_xyz = new Vector<double[]>();
            for (int i = 0; i < degree + 1; i++) {
                // A trivial spline is created with f(t)=(i+i*t+i*t^2,0,0) for all t
                polynomial_xyz.add(new double[] { i, 0., 0. });
            }
            coefficient_array.add(polynomial_xyz);
        }
        InterpolatingPolynomial curve = new InterpolatingPolynomial(coefficient_array,
                interval_length,
                initial_time,
                dimension,
                degree);

        assertEquals(initial_time, curve.initialTime(), 1e-10);
        assertEquals(interval_length, curve.intervalLength(), 1e-10);
        assertEquals(number_of_intervals, curve.numberOfIntervals(), 1e-10);
        assertEquals(5.0, curve.initialTime(), 1e-10);
        for (double t = initial_time; t < initial_time + (number_of_intervals) * interval_length; t += 0.2) {
            double[] value = curve.at(t);
            assertEquals(value[1], 0., 1e-10);
            assertEquals(value[2], 0., 1e-10);
        }
    }

    /**
     * This tests that concatenating two splines creates a spline that has twice the
     * domain
     */
    @Test
    public void testConcatenation() {
        double initial_time = 5.0;
        double interval_length = 2.0;
        int degree = 3;
        int dimension = 3;
        int number_of_intervals = 3;

        // Here the coefficient array for an interpolating polynomial spline is created
        Vector<Vector<double[]>> coefficient_array = new Vector<Vector<double[]>>();
        for (int j = 0; j < number_of_intervals; j++) {
            // This will be the coefficients of a polynomial in R^3 with the basis c_i*t^i
            // for i ranging from 0 to 2
            Vector<double[]> polynomial_xyz = new Vector<double[]>();
            for (int i = 0; i < degree + 1; i++) {
                // A trivial spline is created with f(t)=(i+i*t+i*t^2,0,0) for all t
                polynomial_xyz.add(new double[] { i, 0., 0. });
            }
            coefficient_array.add(polynomial_xyz);
        }
        InterpolatingPolynomial curve = new InterpolatingPolynomial(coefficient_array,
                interval_length,
                initial_time,
                dimension,
                degree);
        InterpolatingPolynomial tail = new InterpolatingPolynomial(coefficient_array,
                interval_length,
                initial_time,
                dimension,
                degree);
        curve.concatenate(tail);
        assertEquals(initial_time, curve.initialTime(), 1e-10);
        assertEquals(interval_length, curve.intervalLength(), 1e-10);
        assertEquals(number_of_intervals + number_of_intervals, curve.numberOfIntervals());
        assertEquals(5.0, curve.initialTime(), 1e-10);
    }

}
