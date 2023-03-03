package team100.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.localization.Blip;
import frc.robot.localization.VisionDataProvider;

public class VisionDataProviderTest {
    private static final double kDelta = 0.01;

    @Test
    public void testBlipToTranslation() {
        // one meter up, two meters left, three meters ahead
        // rotation doesn't matter
        Blip blip = new Blip(5,
                new double[][] {
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 } },
                new double[][] {
                        { -2 },
                        { -1 },
                        { 3 } });

        Translation3d blipTranslation = VisionDataProvider.blipToTranslation(blip);

        // does not change the frame, just copies the values
        assertEquals(-2, blipTranslation.getX(), kDelta);
        assertEquals(-1, blipTranslation.getY(), kDelta);
        assertEquals(3, blipTranslation.getZ(), kDelta);

    }

    @Test
    public void testCameraToNWU() {
        // one meter up, two meters left, three meters ahead
        // rotation doesn't matter
        Blip blip = new Blip(5,
                new double[][] {
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 } },
                new double[][] {
                        { -2 },
                        { -1 },
                        { 3 } });

        Translation3d blipTranslation = VisionDataProvider.blipToTranslation(blip);
        Translation3d nwuTranslation = VisionDataProvider.cameraToNWU(blipTranslation);

        // NWU values now
        assertEquals(3, nwuTranslation.getX(), kDelta);
        assertEquals(2, nwuTranslation.getY(), kDelta);
        assertEquals(1, nwuTranslation.getZ(), kDelta);
    }

    @Test
    public void testBlipToNWU() {
        // one meter up, two meters left, three meters ahead
        // rotation doesn't matter
        Blip blip = new Blip(5,
                new double[][] {
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 } },
                new double[][] {
                        { -2 },
                        { -1 },
                        { 3 } });

        Translation3d nwuTranslation = VisionDataProvider.blipToNWU(blip);

        // NWU values now
        assertEquals(3, nwuTranslation.getX(), kDelta);
        assertEquals(2, nwuTranslation.getY(), kDelta);
        assertEquals(1, nwuTranslation.getZ(), kDelta);
    }

    @Test
    public void testGetRobotPoseInFieldCoords() {
        // trivial example: if camera offset happens to match the camera global pose
        // then of course the robot global pose is the origin.
        Pose3d cameraInFieldCoords = new Pose3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d robotPoseInFieldCoords = VisionDataProvider.getRobotPoseInFieldCoords(
                cameraInFieldCoords,
                cameraInRobotCoords);
        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    public void testGetRobotPoseInFieldCoords2() {
        // trivial example: if camera offset happens to match the camera global pose
        // then of course the robot global pose is the origin.
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));
        Transform3d tagInCameraCoords = new Transform3d(new Translation3d(1, 0, 0), new Rotation3d());
        Pose3d robotPoseInFieldCoords = VisionDataProvider.getRobotPoseInFieldCoords(
                cameraInRobotCoords,
                tagInFieldCoords,
                tagInCameraCoords);

        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    public void testTetRobotPoseInFieldCoords3() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));
        Translation3d tagTranslationInCameraCoords = new Translation3d(1, 0, 0);
        Rotation3d tagRotationInCameraCoords = new Rotation3d(0, 0, 0);
        Pose3d robotPoseInFieldCoords = VisionDataProvider.getRobotPoseInFieldCoords2(
                cameraInRobotCoords,
                tagInFieldCoords,
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);
        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    public void testTetRobotPoseInFieldCoords4() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));

        Rotation3d cameraRotationInFieldCoords = new Rotation3d();

        Blip blip = new Blip(5,
                new double[][] { // pure tilt note we don't actually use this
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 } },
                new double[][] { // one meter range (Z forward)
                        { 0 },
                        { 0 },
                        { 1 } });

        Pose3d robotPoseInFieldCoords = VisionDataProvider.getRobotPoseInFieldCoords(
                cameraInRobotCoords,
                tagInFieldCoords,
                blip,
                cameraRotationInFieldCoords);
        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    // this is the test of the real function
    @Test
    public void testTetRobotPoseInFieldCoords5() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));

        Blip blip = new Blip(5,
                new double[][] { // pure tilt note we don't actually use this
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 } },
                new double[][] { // one meter range (Z forward)
                        { 0 },
                        { 0 },
                        { 1 } });

        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d();

        Pose3d robotPoseInFieldCoords = VisionDataProvider.getRobotPoseInFieldCoords3(
                cameraInRobotCoords,
                tagInFieldCoords,
                blip,
                robotRotationInFieldCoordsFromGyro);

        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    public void testCameraRotationInFieldCoords() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(0.5, 0, 0.5), // front camera
                new Rotation3d(0, -Math.PI / 4, Math.PI / 4)); // pi/4 tilt up, pi/4 yaw left
        // robot rotation should only ever be yaw
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 2);
        Rotation3d cameraRotationInFieldCoords = VisionDataProvider.cameraRotationInFieldCoords(
                cameraInRobotCoords,
                robotRotationInFieldCoordsFromGyro);
        assertEquals(0, cameraRotationInFieldCoords.getX(), kDelta); // still no roll
        assertEquals(-Math.PI / 4, cameraRotationInFieldCoords.getY(), kDelta); // same tilt
        assertEquals(3 * Math.PI / 4, cameraRotationInFieldCoords.getZ(), kDelta);
    }
}
