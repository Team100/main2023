package org.team100.glclib.glc_interface;

import org.team100.glclib.glc_interpolation.InterpolatingPolynomial;

/**
 * The user must define a goal region for the problem instance which
 * informs the algorithm whether or not a trajecoty intersects the goal
 */
public abstract class  GoalRegion {
    /**
     * The user must implement the inGoal method which answers whether traj_
     * intersects the goal and if so, the earliest time at which this happens.
     * 
     * ...
     * 
     * Because java doesn't have outvars, this actually returns -1 as "false"
     * (failed to intersect the goal) and the time >= 0 for "true" (intersect the
     * goal)
     * 
     * @param traj_ the state trajectory that will be checked for
     *              intersection with the goal region
     * @returns the earliest instant at which the
     *          trajectory is in the goal region, or a negative number (-1) if the
     *          trajectory is not within the goal region.
     * 
     */
    public abstract double inGoal(final InterpolatingPolynomial traj_);
};