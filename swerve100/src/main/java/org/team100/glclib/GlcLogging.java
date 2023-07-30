package org.team100.glclib;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import java.util.Set;

import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;

public class GlcLogging {

    /**
     * Logs the states labeling each equivalence class to a nodesToFile
     * 
     * @param name    the desired filename
     * @param path    the desired location for the file to be saved
     * @param domains the set of labeled equivalence classes from a run of
     *                GLC
     */

    public static void nodesToFile(
            final String name,
            final String path,
            final Set<GlcStateEquivalenceClass> domains) {
        try {
            PrintWriter points = new PrintWriter(new FileWriter(path + name));
            for (var x : domains) {
                // NEW! includes a cost column for graph coloring!
                points.print(x.label.cost);
                points.print(",");
                // NEW! includes parent state for line drawing
                GlcNode parent_node = x.label.parent;
                if (parent_node != null) {
                    double[] parent_state = x.label.parent.state;
                    for (int j = 0; j < parent_state.length; j++) {
                        points.print(parent_state[j]);
                        points.print(",");
                    }
                } else { // print the same state twice
                    double[] state = x.label.state;
                    for (int j = 0; j < state.length; j++) {
                        points.print(state[j]);
                        points.print(",");
                    }
                }

                double[] state = x.label.state;
                for (int j = 0; j < state.length - 1; j++) {
                    points.print(state[j]);
                    points.print(",");
                }
                points.println(state[state.length - 1]);
                // TODO: use u_idx to get the control signal out
                //x.label.u_idx

            }
            points.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    /**
     * Logs a finely sampled set of points along a trajectory to a file
     * 
     * @param name       the desired filename
     * @param path       the desired location for the file to be saved
     * @param traj       an interpolating spline object that is to be logged
     * @param num_points the number of points sampled uniformly along traj
     */

    public static void trajectoryToFile(
            final String name,
            final String path,
            final InterpolatingPolynomial traj,
            int num_points) {
        try {
            PrintWriter points = new PrintWriter(new FileWriter(path + name));

            double t = traj.initialTime();
            double dt = (traj.numberOfIntervals() * traj.intervalLength()) / num_points;
            for (int i = 0; i < num_points; i++) {
                double[] state = traj.at(t);
                for (int j = 0; j < state.length - 1; j++) {
                    points.print(state[j]);
                    points.print(",");
                }
                points.println(state[state.length - 1]);
                t += dt;
            }
            points.close();
        } catch (IOException e) {

            e.printStackTrace();

        }

    }

}