package org.team100.glclib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Vector;

import org.junit.jupiter.api.Test;
import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;

public class TestGlcPlannerCore {

    /**
     * This test runs a 2D shortest path problem and checks
     * the solution cost against a gold standard solution
     * from a point where the implementation is believed
     * to be correct.
     */
    @Test
    public void testShortestPathSolution() {
        // Motion planning algorithm parameters
        GlcParameters alg_params = new GlcParameters();
        alg_params.res = 16;
        alg_params.control_dim = 2;
        alg_params.state_dim = 2;
        alg_params.depth_scale = 100;
        alg_params.dt_max = 5.0;
        ////////////////////
        alg_params.max_iter = 50000;
//        alg_params.max_iter = 2;
        alg_params.time_scale = 20;
        alg_params.partition_scale = 40;
        alg_params.x0 = new double[] { 0.0, 0.0 };

        /////////////////
//        System.out.println("START");

        // Create a dynamic model
        SampleInterfaces.SingleIntegrator dynamic_model = new SampleInterfaces.SingleIntegrator(alg_params.dt_max);

        // Create the control inputs
        SampleInterfaces.ControlInputs2D controls = new SampleInterfaces.ControlInputs2D(alg_params.res);
  //      System.out.print("CONTROLS ");
  //      for (int i = 0; i < controls.readInputs().size(); ++i) {
  //          System.out.print(" item " + i + " ");
   //         for (int j = 0; j < controls.readInputs().get(i).length; ++j) {
  //              System.out.print(" array " + j + " ");
  //              System.out.print(controls.readInputs().get(i)[j]);
   //             System.out.print(" ");
  //          }
   //         System.out.println();
   //     }
   //     System.out.println();

        // Create the cost function
        SampleInterfaces.ArcLength performance_objective = new SampleInterfaces.ArcLength(4);

        // Create instance of goal region
        double[] xg = new double[] { 10.0, 10.0 };
        SampleInterfaces.SphericalGoal goal = new SampleInterfaces.SphericalGoal(xg.length, 0.25, 4);
        goal.setGoal(xg);

        // Create the obstacles
        SampleInterfaces.PlanarDemoObstacles obstacles = new SampleInterfaces.PlanarDemoObstacles(4);

        // Create a heuristic for the current goal
        SampleInterfaces.EuclideanHeuristic heuristic = new SampleInterfaces.EuclideanHeuristic(xg, goal.getRadius());
        Planner planner = new Planner(obstacles,
                goal,
                dynamic_model,
                heuristic,
                performance_objective,
                alg_params,
                controls.readInputs());

        /////////////////
    //    System.out.println("PLAN");

        // Run the planner and print solution
        PlannerOutput out = planner.plan();

        assertEquals(out.cost, 14.8374, 1e-3);

    }

    /**
     * This test runs a shortest path problem with a
     * nonholonomic constraint. It checks that a solution
     * is found for this feasible problem, and secondly
     * that the output curve is continuous.
     */
    @Test
    public void testNonholonomicSolutionContinuity() {
        // Motion planning algorithm parameters
        GlcParameters alg_params = new GlcParameters();
        alg_params.res = 21;
        alg_params.control_dim = 2;
        alg_params.state_dim = 3;
        alg_params.depth_scale = 100;
        alg_params.dt_max = 5.0;
        alg_params.max_iter = 50000;
        alg_params.time_scale = 20;
        alg_params.partition_scale = 60;
        alg_params.x0 = new double[] { 0.0, 0.0, Math.PI / 2.0 };

        // Create a dynamic model
        SampleInterfaces.CarNonholonomicConstraint dynamic_model = new SampleInterfaces.CarNonholonomicConstraint(
                alg_params.dt_max);

        // Create the control inputs
        SampleInterfaces.CarControlInputs controls = new SampleInterfaces.CarControlInputs(alg_params.res);

        // Create the cost function
        SampleInterfaces.ArcLength performance_objective = new SampleInterfaces.ArcLength(4);

        // Create instance of goal region
        double goal_radius_sqr = .25;
        double[] goal_center = new double[] { 10.0, 10.0 };
        SampleInterfaces.SphericalGoal2 goal = new SampleInterfaces.SphericalGoal2(goal_radius_sqr, goal_center, 10);

        // Create the obstacles
        SampleInterfaces.PlanarDemoObstacles obstacles = new SampleInterfaces.PlanarDemoObstacles(10);

        // Create a heuristic for the current goal
        SampleInterfaces.EuclideanHeuristic heuristic = new SampleInterfaces.EuclideanHeuristic(goal_center,
                Math.sqrt(goal_radius_sqr));
        Planner planner = new Planner(obstacles,
                goal,
                dynamic_model,
                heuristic,
                performance_objective,
                alg_params,
                controls.readInputs());
        PlannerOutput out = planner.plan();
        assertTrue(out.solution_found);

        Vector<GlcNode> path = planner.pathToRoot(true);
        InterpolatingPolynomial solution = planner.recoverTraj(path);
        double t0 = solution.initialTime();
        double tf = solution.initialTime() + (solution.numberOfIntervals()) * solution.intervalLength();

        for (double t = t0; t < tf; t += (tf - t0) / 1000) {
            double[] x = solution.at(t);
            double[] y = solution.at(t + (tf - t0) / 1000);
            double[] diff = new double[x.length];
            for (int j = 0; j < x.length; ++j) {
                diff[j] = x[j] - y[j];
            }
            assertEquals(GlcMath.norm2(diff), (tf - t0) / 1000, 0.001);
        }
        // GlcLogging.trajectoryToFile("nonholonomic_path_demo.txt","../examples/",solution,500);

    }

    /**
     * This test runs a minimum time pendulum swingup
     * problem and checks that a solution is found and
     * that the solution is continuous
     */
    @Test
    public void testPendulumSolutionContinuity() {
        // Motion planning algorithm parameters
        GlcParameters alg_params = new GlcParameters();
        alg_params.res = 15;
        alg_params.control_dim = 1;
        alg_params.state_dim = 2;
        alg_params.depth_scale = 100;
        alg_params.dt_max = 5.0;
        alg_params.max_iter = 50000;
        alg_params.time_scale = 20;
        alg_params.partition_scale = 40;
        alg_params.x0 = new double[] { 0.0, 0.0 };

        // Create a dynamic model
        SampleInterfaces.InvertedPendulum dynamic_model = new SampleInterfaces.InvertedPendulum(alg_params.dt_max);

        // Create the control inputs
        SampleInterfaces.PendulumTorque controls = new SampleInterfaces.PendulumTorque(alg_params.res);

        // Create the cost function
        SampleInterfaces.MinTime performance_objective = new SampleInterfaces.MinTime(4);

        // Create instance of goal region
        double[] xg = new double[] { Math.PI, 0.0 };
        SampleInterfaces.SphericalGoal goal = new SampleInterfaces.SphericalGoal(xg.length, 1.0, 4);
        goal.setGoal(xg);

        // Create the obstacles
        SampleInterfaces.UnconstrainedSpace obstacles = new SampleInterfaces.UnconstrainedSpace(4);

        // Create a heuristic for the current goal
        SampleInterfaces.ZeroHeuristic heuristic = new SampleInterfaces.ZeroHeuristic();
        Planner planner = new Planner(obstacles,
                goal,
                dynamic_model,
                heuristic,
                performance_objective,
                alg_params,
                controls.readInputs());

        // Run the planner and print solution
        PlannerOutput out = planner.plan();
        assertTrue(out.solution_found);

        int count = 0;
        Vector<GlcNode> path = planner.pathToRoot(true);
        for (GlcNode n : path) {
            if (count == 0) {
                continue;
            }

            double t0 = n.trajectory_from_parent.initialTime();
            double dt = n.trajectory_from_parent.intervalLength() * (n.trajectory_from_parent.numberOfIntervals());
            double tf = t0 + dt;
            var xf = n.trajectory_from_parent.at(tf);
            count++;
        }
        InterpolatingPolynomial solution = planner.recoverTraj(path);
        double t0 = solution.initialTime();
        double tf = solution.initialTime() + (solution.numberOfIntervals()) * solution.intervalLength();

        for (double t = t0; t < tf; t += (tf - t0) / 100) {
            double[] x = solution.at(t);
            double[] y = solution.at(t + (tf - t0) / 100);
            double[] diff = new double[x.length];
            for (int j = 0; j < x.length; ++j) {
                diff[j] = x[j] - y[j];
            }
            assertTrue(GlcMath.norm2(diff) <= 3.0 * (tf - t0) / 100);
            // std::cout << GlcMath.norm2(x-y) << std::endl;
        }
//        GlcLogging.trajectoryToFile("pendulum_swingup_demo.txt", "../examples/", solution, 100);
        //GlcLogging.trajectoryToFile("pendulum_swingup_demo.txt", "", solution, 100);

    }

}
