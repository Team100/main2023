package team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.localization.Blip;
import frc.robot.localization.VisionDataProvider;

/**
 * Remind myself how to correct for the camera offset, pan, tilt, etc.
 */
public class VisionTransformTest {
    private static final double kDelta = 0.01;

    @Test
    public void testNoOffsets() throws IOException {
        Path path = Filesystem.getDeployDirectory().toPath().resolve("2023-chargedup.json");
        AprilTagFieldLayout layout = new AprilTagFieldLayout(path);
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        // tag 5 is at (0.36, 6.75, 0.69) from the blue perspective, i.e. in the near
        // left corner.
        // ignoring camera offsets for now, put the camera directly in front of the
        // tag, at (1.36, 6,75, 0.69).
        // this should yield the following Blip from the camera:
        Blip blip = new Blip(5,
                new double[][] { // no rotation
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 } },
                new double[][] { // one meter range
                        { 1 },
                        { 0 },
                        { 0 } });
        Pose3d tagInCameraCoords = VisionDataProvider.blipToPose(blip);
        assertEquals(1, tagInCameraCoords.getTranslation().getX(), kDelta); // same as above
        assertEquals(0, tagInCameraCoords.getTranslation().getY(), kDelta);
        assertEquals(0, tagInCameraCoords.getTranslation().getZ(), kDelta);
        // no rotations
        assertEquals(0, tagInCameraCoords.getRotation().getX(), kDelta);
        assertEquals(0, tagInCameraCoords.getRotation().getY(), kDelta);
        assertEquals(0, tagInCameraCoords.getRotation().getZ(), kDelta);

        Pose3d tagInRobotCoords = VisionDataProvider.cameraToRobot("", tagInCameraCoords);
        assertEquals(0, tagInRobotCoords.getTranslation().getX(), kDelta);
        assertEquals(0, tagInRobotCoords.getTranslation().getY(), kDelta);
        assertEquals(0, tagInRobotCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, tagInRobotCoords.getRotation().getX(), kDelta);
        assertEquals(0, tagInRobotCoords.getRotation().getY(), kDelta);
        assertEquals(0, tagInRobotCoords.getRotation().getZ(), kDelta);

        Pose3d tagInFieldCoords = layout.getTagPose(blip.id).get();
        assertEquals(0, tagInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, tagInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, tagInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, tagInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, tagInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, tagInFieldCoords.getRotation().getZ(), kDelta);

        Pose3d robotInFieldCoords = VisionDataProvider.toFieldCoordinates(
                new Pose2d(),
                tagInRobotCoords.getTranslation(),
                tagInRobotCoords.getRotation(),
                tagInFieldCoords);
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getZ(), kDelta);

        Pose2d robotIn2dFieldCords = robotInFieldCoords.toPose2d();
        assertEquals(0, robotIn2dFieldCords.getTranslation().getX(), kDelta);
        assertEquals(0, robotIn2dFieldCords.getTranslation().getY(), kDelta);
        assertEquals(0, robotIn2dFieldCords.getRotation().getRadians(), kDelta);
    }

}
