package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autonomous.DriveToAprilTag;
import team100.localization.AprilTagFieldLayoutWithCorrectOrientation;

public class goalTest {

    @Test
    public void testRedSubstation() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation m_layout = AprilTagFieldLayoutWithCorrectOrientation.redLayout();
        Pose2d m_goal = DriveToAprilTag.goal(5, 1, m_layout);
        assertEquals(m_goal.getX(), 15.18, .001);
        assertEquals(m_goal.getY(), 1.46, .005);
        assertEquals(m_goal.getRotation().getRadians(), 0, 0.001);
    }

    @Test
    public void testBlueSubstation() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation m_layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout();
        Pose2d m_goal = DriveToAprilTag.goal(4, 1, m_layout);
        assertEquals(m_goal.getX(), 15.18, .01);
        assertEquals(m_goal.getY(), 6.95, .005);
        assertEquals(m_goal.getRotation().getRadians(), 0, 0.001);
    }

}
