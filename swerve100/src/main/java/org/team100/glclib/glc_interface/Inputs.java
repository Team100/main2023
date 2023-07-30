package org.team100.glclib.glc_interface;

import java.util.Arrays;
import java.util.Vector;

/**
 * Base class which defines a finite set of control inputs from the input
 * space that are used by the planner to forward simulate the system dynamics
 * 
 * Mathematically, the GLC method produces an output converting to a globally
 * optimal solution for a particular problem instance as the resolution of the
 * algorithm is increased. The user is responsible for implementing a derived
 * class for Inputs which is parameterized by the algorithm resolution such
 * that with increasing resolution, the dispersion of the discrete set of
 * control inputs within the set of admissible control inputs converges to
 * zero.
 */
public class Inputs {
    // stores the finite set of control inputs
    private final Vector<double[]> points = new Vector<double[]>();

    /**
     * Inserts a control input to the set of control inputs
     * 
     * Two simple ways of generating a set of control inputs meeting the
     * requirements of the algorithm are to sample randomly from the set of
     * admissible control inputs with the number of samples increasing with
     * resolution or to intersect a uniform grid with the set of admissible control
     * inputs and refine the grid with increasing algorithm resolution.
     * 
     * @param _input the control input added to the set of inputs
     */
    protected void addInputSample(double[] _input) {
        // copy here because original made a copy
        points.add(Arrays.copyOf(_input, _input.length));
    }

    /**
     * @returns a copy of the set of control inputs
     */
    public Vector<double[]> readInputs() {
        return points;
    }
};