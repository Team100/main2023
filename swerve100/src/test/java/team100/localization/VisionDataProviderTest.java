package team100.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionDataProviderTest {
    private static final double kDelta = 0.01;

    @Test
    public void testGetRobotPoseInFieldCoords2() {
        // trivial example: if camera offset happens to match the camera global pose
        // then of course the robot global pose is the origin.
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));
        Transform3d tagInCameraCoords = new Transform3d(new Translation3d(1, 0, 0), new Rotation3d());
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        Pose3d robotPoseInFieldCoords = PoseEstimationHelper.applyCameraOffset(
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
    public void testGetRobotPoseInFieldCoords3() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInFieldCoords = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));
        Translation3d tagTranslationInCameraCoords = new Translation3d(1, 0, 0);
        Rotation3d tagRotationInCameraCoords = new Rotation3d(0, 0, 0);
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        Pose3d robotPoseInFieldCoords = PoseEstimationHelper.applyCameraOffset(
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
    public void testGetRobotPoseInFieldCoords4() {
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
        Translation3d tagTranslationInCameraCoords = PoseEstimationHelper.blipToNWU(blip);
        Rotation3d tagRotationInCameraCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);
        Pose3d cameraInFieldCoords = PoseEstimationHelper.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);

        Pose3d robotPoseInFieldCoords = PoseEstimationHelper.applyCameraOffset(
                cameraInFieldCoords,
                cameraInRobotCoords);
        assertEquals(0, robotPoseInFieldCoords.getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getZ(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInFieldCoords.getRotation().getZ(), kDelta);
    }

    // this is the test of the real function
    @Test
    public void testGetRobotPoseInFieldCoords5() {
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

        Pose3d robotPoseInFieldCoords = VisionDataProvider.getRobotPoseInFieldCoords(
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

}
