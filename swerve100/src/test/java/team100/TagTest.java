package team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * remind myself what's in the JSON file.
 */
public class TagTest {
    private static final double kDelta = 0.01;

    @Test
    public void testBlueLayout() throws IOException {
        Path path = Filesystem.getDeployDirectory().toPath().resolve("2023-chargedup.json");
        AprilTagFieldLayout layout = new AprilTagFieldLayout(path);
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        /*
         * from the blue perspective, tag 5 has small x
         * and large y, and the "front" is oriented facing zero theta;
         * note the real "front" might be the back?
         */
        Pose3d tag5Pose = layout.getTagPose(5).get();
        assertEquals(0.36, tag5Pose.getTranslation().getX(), kDelta); // close to baseline
        assertEquals(6.75, tag5Pose.getTranslation().getY(), kDelta); // far to left
        assertEquals(0.69, tag5Pose.getTranslation().getZ(), kDelta); // 2 feet up
        // no rotations, i.e. facing away from us, zero degrees
        assertEquals(0, tag5Pose.getRotation().getX(), kDelta);
        assertEquals(0, tag5Pose.getRotation().getY(), kDelta);
        assertEquals(0, tag5Pose.getRotation().getZ(), kDelta);
    }

    @Test
    public void testRedLayout() throws IOException {
        Path path = Filesystem.getDeployDirectory().toPath().resolve("2023-chargedup.json");
        AprilTagFieldLayout layout = new AprilTagFieldLayout(path);
        layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        /*
         * from the red perspective, tag 5 has large x
         * and small y, and the "front" is oriented facing 180 degrees;
         * note the real "front" might be the back?
         */
        Pose3d tag5Pose = layout.getTagPose(5).get();
        assertEquals(16.18, tag5Pose.getTranslation().getX(), kDelta); // far ahead
        assertEquals(1.26, tag5Pose.getTranslation().getY(), kDelta); // close to right side
        assertEquals(0.69, tag5Pose.getTranslation().getZ(), kDelta); // 2 feet up (as above)
        // no rotations
        assertEquals(0, tag5Pose.getRotation().getX(), kDelta);
        assertEquals(0, tag5Pose.getRotation().getY(), kDelta);
        // facing towards us, i.e. 180 degrees
        assertEquals(-Math.PI, tag5Pose.getRotation().getZ(), kDelta);
    }

}
