package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.HolonomicDriveRegulator;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class HolonomicDriveRegulatorTest {
    private HolonomicDriveRegulator m_regulator = new HolonomicDriveRegulator();
    private Pose2d currentPose;
    private SwerveState desiredState;
    private final SpeedLimits speedLimits = new SpeedLimits(5, 2, 2, 2);
    private MotionProfile profileX;
    private MotionProfile profileY;
    private MotionProfile profileTheta;
    private double time;
    private double kDelta = .01;

    @Test
    void testAtSetpoint() {
        currentPose = new Pose2d();
        desiredState = new SwerveState(new State100(0, 0, 0), new State100(0, 0, 0), new State100(0, 0, 0));
        Twist2d output = m_regulator.calculate(currentPose, desiredState);
        System.out.println(output);
        assertEquals(output.dx, 0);
        assertEquals(output.dy, 0);
        assertEquals(output.dtheta, 0);
    }

    @Test
    void driveOneMeter() {
        testWithTrajectory(new Pose2d(), new Pose2d(1, 0, new Rotation2d(0)));
        assertTrue(true);
    }

    void testWithTrajectory(Pose2d startingPose, Pose2d goalPose) {
        currentPose = startingPose;
        time = 0;
        profileX = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(startingPose.getX(), 0),
                new MotionState(goalPose.getX(), 0),
                speedLimits.speedM_S,
                speedLimits.accelM_S2,
                speedLimits.jerkM_S3);
        profileY = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(startingPose.getY(), 0),
                new MotionState(goalPose.getY(), 0),
                speedLimits.speedM_S,
                speedLimits.accelM_S2,
                speedLimits.jerkM_S3);
        profileTheta = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(startingPose.getRotation().getRadians(), 0),
                new MotionState(goalPose.getRotation().getRadians(), 0),
                speedLimits.angleSpeedRad_S,
                speedLimits.angleAccelRad_S2,
                speedLimits.angleJerkRad_S3);

        double duration = Math.max(profileX.duration(), Math.max(profileY.duration(), profileTheta.duration()));
        System.out.println(duration);

        while (time < duration) {
            desiredState = new SwerveState(new State100(profileX.get(time)), new State100(profileY.get(time)),
                    new State100(profileTheta.get(time)));
            Twist2d output = m_regulator.calculate(currentPose, desiredState);
            if (time == 0) {
                output = new Twist2d();
            }
            currentPose = new Pose2d(currentPose.getX() + output.dx * .05, currentPose.getY() + output.dy * .05,
                    new Rotation2d(currentPose.getRotation().getRadians() + output.dtheta * .05));
            System.out.println("\noutput: " + output);
            System.out.println("\ncurrent pose: " + currentPose);
            System.out.println("\ndesired state: " + desiredState);
            time += .05;
        }

        assertEquals(currentPose.getX(), desiredState.x().x(), kDelta);
        assertEquals(currentPose.getY(), desiredState.y().x(), kDelta);
        assertEquals(currentPose.getRotation().getRadians(), desiredState.theta().x(), kDelta);
    }
}
