package org.team100.glclib.glc_interpolation;

import java.util.Arrays;
import java.util.Vector;

/**
 * A class used to represent parametric curves in arbitrary dimensions
 * 
 * This is a very minimal class that does not enforce continuity or smoothness
 * of the curve it represents. This give maximum flexibility to the user. A
 * series of polynomials are defined on evenly spaced time intervals using
 * the standard monomial basis [1, t, t^2, t^3, ... ,t^(degree-1)]
 */
public class InterpolatingPolynomial {
    /**
     * the dimension of the polynomial vector space spanned by the basis
     * 
     * Node that degree is a slight misnomer. It represents the number of
     * coefficients
     * required in each state dimension to define a polynomial. The highest power
     * appearing in the basis functions is degree-1.
     */
    private final int degree;

    /**
     * dimension is the dimension of the state space that the curve is
     * defined in
     * 
     * There is a polynomial assigned to each dimension for a given curve
     */
    private final int dimension;

    /**
     * This is the time interval for each polynomial segment
     * 
     * Each polynomial is defined from t=0 to t=collocation_interval.
     */
    private final double collocation_interval;

    /**
     * This is the initial time the curve is defined at
     */
    private final double t0;

    /**
     * The 3D array of coefficients for the muli-dimensional interpolating
     * spline
     * 
     * The outermost index identifies the collocation interval. The middle index
     * identifies the polynomial basis monomial index. Tha innermost index
     * identifies the coordinate in the state space. In summary,
     * 
     * coefficient_array[time_interval_index][polynomial_coefficient_index][polynomial_coordinate_index]
     */
    private final Vector<Vector<double[]>> coefficient_array;

    public InterpolatingPolynomial(final Vector<Vector<double[]>> _coeff_array,
            final double _collocation_interval,
            final double _t0,
            final int _dimension,
            final int _degree) {
        collocation_interval = _collocation_interval;
        t0 = _t0;
        dimension = _dimension;
        degree = _degree;
        coefficient_array = _coeff_array;

    }

    /**
     * This constructor takes values to set member attributes with the
     * exception of the coefficients which are left empty
     * 
     * @param _collocation_interval initializes _collocation_interval
     * @param _t0                   initialilzed t0
     * @param _dimension            initializes dimension
     * @param _degree               initializes degree
     */
    public InterpolatingPolynomial(final double _collocation_interval,
            final double _t0,
            final int _dimension,
            final int _degree) {
        collocation_interval = _collocation_interval;
        t0 = _t0;
        dimension = _dimension;
        degree = _degree;
        coefficient_array = new Vector<Vector<double[]>>();
    }

    /**
     * Extends this curve by copying the contents of tail to the back of this
     * curve
     * 
     * The value of t0 in tail is ignored
     */
    public void concatenate(final InterpolatingPolynomial tail) {
        if (tail == null)
            throw new IllegalArgumentException();
        if (tail.dimension != dimension)
            throw new IllegalArgumentException();
        if (tail.degree != degree)
            throw new IllegalArgumentException();
        if (Math.abs(tail.collocation_interval - collocation_interval) > 1e-4)
            throw new IllegalArgumentException();
        coefficient_array.addAll(tail.coefficient_array);
    }

    /**
     * appends a single polynomial segment to the back of this spline
     */
    void push(final Vector<double[]> knot) {
        // make a copy here because the original made a copy
        Vector<double[]> out = new Vector<double[]>();
        for (int i = 0; i < knot.size(); ++i) {
            out.add(Arrays.copyOf(knot.get(i), knot.get(i).length));
        }
        coefficient_array.add(out);
    }

    /**
     * Evaluate the curve at parameter value t
     * 
     * This method first identifies which interval I should
     * be evaluated. From there an offset equal to t-I*collocation_interval
     * is computed and this value is passed to the monomial basis on the
     * inner array indices of coefficient_array.
     */
    public double[] at(final double t) {
        int index = Math.min((int) coefficient_array.size() - 1,
                Math.max(0, (int) Math.floor((t - t0) / collocation_interval)));
        double time = (t - t0) - collocation_interval * ((double) index);
        double[] eval = new double[dimension];
        for (int i = 0; i < degree; i++) {
            for (int j = 0; j < dimension; ++j) {
                eval[j] += coefficient_array.get(index).get(i)[j] * Math.pow(time, i);
            }
        }
        return eval;
    }

    /**
     * Allocates memory in coefficient_array for "size" spline intervals
     */
    public void reserve(final int size) {
        if (size < 0)
            throw new IllegalArgumentException(
                    "Cannot reserve negative space for InterpolatingPolynomial coefficients");
        coefficient_array.ensureCapacity(size);
    }

    /**
     * the number of intervals in this interpolating spline
     */
    public int numberOfIntervals() {
        return coefficient_array.size();
    }

    /**
     * the length of each interval -- each interval has equal length
     */
    public double intervalLength() {
        return collocation_interval;
    }

    /**
     * the initial time for the parametric curve
     */
    public double initialTime() {
        return t0;
    }

    /**
     * prints uniformly sampled points along the curve to the terminal
     * 
     * @param num_points
     * @param msg        is a message that will accompany the output to the terminal
     */
    public void printSpline(int num_points, final String msg) {
        System.out.println("\n*****" + msg + "*****");
        double t = initialTime();
        double dt = (intervalLength() * numberOfIntervals()) / (double) num_points;
        for (int i = 0; i < num_points + 1; i++) {
            double[] x = (at(t));
            System.out.print("(");
            for (int j = 0; j < x.length - 1; j++) {
                System.out.print(x[j]);
                System.out.print(",");
            }
            System.out.print(x[x.length - 1]);
            System.out.println(")");
            t += dt;
        }
    }

    public void printData() {
        for (int t_id = 0; t_id < numberOfIntervals(); t_id++) {
            System.out.println("\n==== interval: " + t_id + "====");
            for (int mon_id = 0; mon_id < 4; mon_id++) {
                System.out.print("--- t^" + mon_id + ": ");
                for (int s_id = 0; s_id < 2; s_id++) {
                    Vector<double[]> item = coefficient_array.get(t_id);
                    if (item == null)
                        continue;
                    if (item.size() - 1 < mon_id)
                        continue;
                    double[] arr = item.get(mon_id);
                    System.out.print(arr[s_id]);
                    System.out.print(",");
                }
            }
            System.out.print("end: " + at(initialTime() + (t_id + 1) * intervalLength())[0]
                    + "," + at(initialTime() + (t_id + 1) * intervalLength())[1]);
        }

    }

}