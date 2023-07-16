package com.team254.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.trajectory.TrajectoryPoint;
import com.team254.lib.util.Util;

public class SplineGeneratorTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        // Create the test spline
        Pose2d p1 = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        Pose2d p2 = new Pose2d(new Translation2d(15, 10), new Rotation2d(1, -5, true));
        Spline s = new QuinticHermiteSpline(p1, p2);
        List<Rotation2d> headings = List.of(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90));

        List<TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> samples = SplineGenerator.parameterizeSpline(s, headings);

        double arclength = 0;
        Rotation2d cur_heading = Rotation2d.identity();
        Pose2dWithCurvature cur_pose = samples.get(0).state();
        for (TrajectoryPoint<Pose2dWithCurvature, Rotation2d> point : samples) {
            Pose2dWithCurvature sample = point.state();
            final Twist2d t = Pose2d.log(cur_pose.getPose().inverse().transformBy(sample.getPose()));
            arclength += t.dx;
            cur_pose = sample;
            cur_heading = point.heading();
        }

        assertEquals(cur_pose.getTranslation().x(), 15.0, kTestEpsilon);
        assertEquals(cur_pose.getTranslation().y(), 10.0, kTestEpsilon);
        assertEquals(cur_pose.getRotation().getDegrees(), -78.69006752597981, kTestEpsilon);
        assertEquals(arclength, 23.17291953186379, kTestEpsilon);
        assertEquals(cur_heading.getRadians(), headings.get(1).getRadians(), kTestEpsilon);
    }
}
