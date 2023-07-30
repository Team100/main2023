package org.team100.glclib.glc_interface;

import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;

/**
 * The user must define the infeasible space for the problem instance to inform the algorithm whether or not a trajectory is feasible
 */
public abstract class Obstacles{
      // collision_counter monitors the number of times the collisionFree method is called in a planning query
      public int collision_counter=0;
      /**
       *  The user must implement the collisionFree method for their problem instance
       * @param traj_ is the state trajectory that will be checked for intersection with the infeasible region for a particular problem
       * @returns The method returns true of the trajectory remains in the feasible region (i.e. it is collision free) and false otherwise
       */
      public abstract boolean collisionFree(final InterpolatingPolynomial traj_); 
    };