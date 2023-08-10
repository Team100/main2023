package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.HolonomicDriveRegulator;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.math.geometry.Pose2d;
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

    void testWithTrajectory(Pose2d startingPose, Pose2d goalPose) {
        currentPose = startingPose;
        time = 0;
        profileX = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(startingPose.getX(), 0),
            new MotionState(goalPose.getX() , 0),
            speedLimits.speedM_S,
            speedLimits.accelM_S2,
            speedLimits.jerkM_S3);
        profileY = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(startingPose.getY(), 0),
            new MotionState(goalPose.getY() , 0),
            speedLimits.speedM_S,
            speedLimits.accelM_S2,
            speedLimits.jerkM_S3);
        profileTheta = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(startingPose.getRotation().getRadians(), 0),
            new MotionState(goalPose.getRotation().getRadians(), 0),
            speedLimits.angleSpeedRad_S,
            speedLimits.angleAccelRad_S2,
            speedLimits.angleJerkRad_S3);
    }
    double duration = Math.max(profileX.duration(), Math.max(profileY.duration(), profileTheta.duration()));
}
