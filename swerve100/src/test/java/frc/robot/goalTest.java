package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.autonomous.DriveToAprilTag;
import frc.robot.localization.AprilFieldLayout2;
import frc.robot.localization.JSONFile;

public class goalTest {

    @Test
    public void testRedSubstation() throws IOException {
        AprilTagFieldLayout m_layout = new AprilTagFieldLayout(JSONFile.getPath());
        m_layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        Pose2d m_goal = DriveToAprilTag.goal(5, m_layout);
        assertEquals(m_goal.getX(), 15.18, .001);
        assertEquals(m_goal.getY(), 1.26, .005);
        assertEquals(m_goal.getRotation().getRadians(), 0, 0.001);
    }
    @Test
    public void testBlueSubstation() throws IOException {
        AprilTagFieldLayout m_layout = new AprilTagFieldLayout(JSONFile.getPath());
        m_layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        Pose2d m_goal = DriveToAprilTag.goal(4, m_layout);
        assertEquals(m_goal.getX(), 15.18, .01);
        assertEquals(m_goal.getY(), 6.75, .005);
        assertEquals(m_goal.getRotation().getRadians(), 0, 0.001);
    }

}
