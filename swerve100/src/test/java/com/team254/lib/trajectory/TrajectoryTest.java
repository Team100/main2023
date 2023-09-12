package com.team254.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;

public class TrajectoryTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    public static final List<Translation2d> kWaypoints = Arrays.asList(
            new Translation2d(0.0, 0.0),
            new Translation2d(24.0, 0.0),
            new Translation2d(36.0, 12.0),
            new Translation2d(60.0, 12.0));

    List<Rotation2d> kHeadings = Arrays.asList(
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(30),
            Rotation2d.fromDegrees(60),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(180));

    @Test
    public void testConstruction() {
        // Empty constructor.
        Trajectory<Translation2d, Rotation2d> traj = new Trajectory<>();
        assertTrue(traj.isEmpty());
        assertEquals(0.0, traj.getIndexView().first_interpolant(), kTestEpsilon);
        assertEquals(0.0, traj.getIndexView().last_interpolant(), kTestEpsilon);
        assertEquals(0, traj.length());

        // Set states at construction time.
        traj = new Trajectory<>(kWaypoints, kHeadings);
        assertFalse(traj.isEmpty());
        assertEquals(0.0, traj.getIndexView().first_interpolant(), kTestEpsilon);
        assertEquals(3.0, traj.getIndexView().last_interpolant(), kTestEpsilon);
        assertEquals(4, traj.length());
    }

    @Test
    public void testStateAccessors() {
        Trajectory<Translation2d, Rotation2d> traj = new Trajectory<>(kWaypoints, kHeadings);

        assertEquals(kWaypoints.get(0), traj.getPoint(0).state());
        assertEquals(kWaypoints.get(1), traj.getPoint(1).state());
        assertEquals(kWaypoints.get(2), traj.getPoint(2).state());
        assertEquals(kWaypoints.get(3), traj.getPoint(3).state());

        assertEquals(kHeadings.get(0), traj.getPoint(0).heading());
        assertEquals(kHeadings.get(1), traj.getPoint(1).heading());
        assertEquals(kHeadings.get(2), traj.getPoint(2).heading());
        assertEquals(kHeadings.get(3), traj.getPoint(3).heading());

        assertEquals(kWaypoints.get(0), traj.getInterpolated(0.0).state());
        assertEquals(traj.getInterpolated(0.0).index_floor(), 0);
        assertEquals(traj.getInterpolated(0.0).index_ceil(), 0);
        assertEquals(kWaypoints.get(1), traj.getInterpolated(1.0).state());
        assertEquals(traj.getInterpolated(1.0).index_floor(), 1);
        assertEquals(traj.getInterpolated(1.0).index_ceil(), 1);
        assertEquals(kWaypoints.get(2), traj.getInterpolated(2.0).state());
        assertEquals(traj.getInterpolated(2.0).index_floor(), 2);
        assertEquals(traj.getInterpolated(2.0).index_ceil(), 2);
        assertEquals(kWaypoints.get(3), traj.getInterpolated(3.0).state());
        assertEquals(traj.getInterpolated(3.0).index_floor(), 3);
        assertEquals(traj.getInterpolated(3.0).index_ceil(), 3);

        assertEquals(kHeadings.get(0), traj.getInterpolated(0.0).heading());
        assertEquals(traj.getInterpolated(0.0).index_floor(), 0);
        assertEquals(traj.getInterpolated(0.0).index_ceil(), 0);
        assertEquals(kHeadings.get(1), traj.getInterpolated(1.0).heading());
        assertEquals(traj.getInterpolated(1.0).index_floor(), 1);
        assertEquals(traj.getInterpolated(1.0).index_ceil(), 1);
        assertEquals(kHeadings.get(2), traj.getInterpolated(2.0).heading());
        assertEquals(traj.getInterpolated(2.0).index_floor(), 2);
        assertEquals(traj.getInterpolated(2.0).index_ceil(), 2);
        assertEquals(kHeadings.get(3), traj.getInterpolated(3.0).heading());
        assertEquals(traj.getInterpolated(3.0).index_floor(), 3);
        assertEquals(traj.getInterpolated(3.0).index_ceil(), 3);

        assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), .25), traj.getInterpolated(0.25).state());
        assertEquals(traj.getInterpolated(0.25).index_floor(), 0);
        assertEquals(traj.getInterpolated(0.25).index_ceil(), 1);
        assertEquals(kWaypoints.get(1).interpolate(kWaypoints.get(2), .5), traj.getInterpolated(1.5).state());
        assertEquals(traj.getInterpolated(1.5).index_floor(), 1);
        assertEquals(traj.getInterpolated(1.5).index_ceil(), 2);
        assertEquals(kWaypoints.get(2).interpolate(kWaypoints.get(3), .75), traj.getInterpolated(2.75).state());
        assertEquals(traj.getInterpolated(2.75).index_floor(), 2);
        assertEquals(traj.getInterpolated(2.75).index_ceil(), 3);

        assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), .25), traj.getInterpolated(0.25).heading());
        assertEquals(traj.getInterpolated(0.25).index_floor(), 0);
        assertEquals(traj.getInterpolated(0.25).index_ceil(), 1);
        assertEquals(kHeadings.get(1).interpolate(kHeadings.get(2), .5), traj.getInterpolated(1.5).heading());
        assertEquals(traj.getInterpolated(1.5).index_floor(), 1);
        assertEquals(traj.getInterpolated(1.5).index_ceil(), 2);
        assertEquals(kHeadings.get(2).interpolate(kHeadings.get(3), .75), traj.getInterpolated(2.75).heading());
        assertEquals(traj.getInterpolated(2.75).index_floor(), 2);
        assertEquals(traj.getInterpolated(2.75).index_ceil(), 3);

        Trajectory<Translation2d, Rotation2d>.IndexView index_view = traj.getIndexView();
        assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), .25), index_view.sample(0.25).state());
        assertEquals(kWaypoints.get(1).interpolate(kWaypoints.get(2), .5), index_view.sample(1.5).state());
        assertEquals(kWaypoints.get(2).interpolate(kWaypoints.get(3), .75), index_view.sample(2.75).state());
        assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), .25), index_view.sample(0.25).heading());
        assertEquals(kHeadings.get(1).interpolate(kHeadings.get(2), .5), index_view.sample(1.5).heading());
        assertEquals(kHeadings.get(2).interpolate(kHeadings.get(3), .75), index_view.sample(2.75).heading());
    }
}
