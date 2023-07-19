package org.team100.frc2023.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.HeadingInterface;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;

/** Example of mock objects for testing. */
public class RotateTest {
    private static final double kDelta = 0.001;

    class MockHeading implements HeadingInterface {
        @Override
        public Rotation2d getHeading() {
            return new Rotation2d();
        }
    }

    class MockSwerveDriveSubsystem implements SwerveDriveSubsystemInterface {

        Pose2d pose = new Pose2d();
        double output = 0;
        boolean stopped = false;

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public void stop() {
            stopped = true;
        }

        @Override
        public void driveInRobotCoords(Twist2d twist2d) {
            output = twist2d.dtheta;
        }
    }

    class MockTimer extends Timer {

        double time = 0;

        @Override
        public void reset() {
            time = 0;
        }

        @Override
        public double get() {
            return time;
        }

    }

    @Test
    public void testRotate() {
        MockSwerveDriveSubsystem swerveDriveSubsystem = new MockSwerveDriveSubsystem();
        swerveDriveSubsystem.pose = new Pose2d();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        PIDController rotateController = new PIDController(1, 0, 0);
        rotateController.enableContinuousInput(-Math.PI, Math.PI);
        rotateController.setTolerance(0.003, 0.003); // one degree, one degree per second
        MockTimer timer = new MockTimer();
        double targetAngle = Math.PI / 2;
        Rotate rotate = new Rotate(
                swerveDriveSubsystem,
                speedLimits,
                rotateController,
                timer,
                targetAngle);

        timer.time = 100; // initial time is not zero

        rotate.initialize();

        assertEquals(0, timer.time, kDelta); // now the timer is reset
        assertEquals(0, rotate.profile.start().getX(), kDelta);
        assertEquals(Math.PI/2, rotate.profile.end().getX(), kDelta);
        assertEquals(2.571, rotate.profile.duration(), kDelta);

        rotate.execute();

        assertEquals(0, rotate.reference.getX(), kDelta); // at start
        assertEquals(0, swerveDriveSubsystem.output, kDelta);

        timer.time = 1;
        rotate.execute();
        assertEquals(0.5, rotate.reference.getX(), kDelta);
        assertEquals(1.5, swerveDriveSubsystem.output, kDelta);

        timer.time = 2;
        swerveDriveSubsystem.pose = new Pose2d(0, 0, new Rotation2d(1));
        rotate.execute();
        assertEquals(1.408, rotate.reference.getX(), kDelta);
        assertEquals(0.979, swerveDriveSubsystem.output, kDelta);

        timer.time = 3;
        swerveDriveSubsystem.pose = new Pose2d(0, 0, new Rotation2d(Math.PI));

        assertFalse(swerveDriveSubsystem.stopped);
        rotate.end(false);
        assertTrue(swerveDriveSubsystem.stopped);

    }
}
