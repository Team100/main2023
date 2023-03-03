package team100.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Remind myself how to correct for the camera offset, pan, tilt, etc.
 */
public class VisionTransformTest {
    private static final double kDelta = 0.01;

    /*
     * This test uses the blue perspective.
     * The robot/camera is looking back at the baseline, one meter in front of tag
     * 5, so at R(0,0,pi)|t(1.36, 6.75, 0.69)
     * Tag 5 is on "our" side, at R(pi)|t(0.36, 6.75, 0.69), using the
     * "into the page" orientation
     */
    @Test
    public void testNoOffsets() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout();
        // this is what the camera sees
        Blip blip = new Blip(5,
                new double[][] { // no rotation
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 } },
                new double[][] { // one meter range (Z forward)
                        { 0 },
                        { 0 },
                        { 1 } });
        // camera is z-forward
        Pose3d tagInCameraCoords = VisionDataProvider.blipToPose(blip);
        assertEquals(0, tagInCameraCoords.getTranslation().getX(), kDelta); // same as above, 1m
        assertEquals(0, tagInCameraCoords.getTranslation().getY(), kDelta);
        assertEquals(1, tagInCameraCoords.getTranslation().getZ(), kDelta);
        // no rotations, unlike the json
        assertEquals(0, tagInCameraCoords.getRotation().getX(), kDelta);
        assertEquals(0, tagInCameraCoords.getRotation().getY(), kDelta);
        assertEquals(0, tagInCameraCoords.getRotation().getZ(), kDelta);

        // transform to x-forward
        Pose3d tagInRobotCoords = VisionDataProvider.cameraToRobot("", tagInCameraCoords);
        assertEquals(1, tagInRobotCoords.getTranslation().getX(), kDelta); // one meter ahead, X forward
        assertEquals(0, tagInRobotCoords.getTranslation().getY(), kDelta);
        assertEquals(0, tagInRobotCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, tagInRobotCoords.getRotation().getX(), kDelta);
        assertEquals(0, tagInRobotCoords.getRotation().getY(), kDelta);
        assertEquals(0, tagInRobotCoords.getRotation().getZ(), kDelta);

        // "into the page" orientation
        Pose3d tagInFieldCoords = layout.getTagPose(blip.id).get();
        assertEquals(0.36, tagInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(6.75, tagInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0.69, tagInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, tagInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, tagInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI, tagInFieldCoords.getRotation().getZ(), kDelta); // <= fixed

        // this is actually *camera* in field coords
        // in this setup the robot is facing toward negative X
        Rotation3d gyro = new Rotation3d(0, 0, Math.PI);
        Rotation3d realTagRotationInRobotCoords = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(), gyro);

        Pose3d robotInFieldCoords = VisionDataProvider.toFieldCoordinates( // <== THIS IS WHERE THE BUG IS
                realTagRotationInRobotCoords,
                tagInRobotCoords.getTranslation(),
                tagInFieldCoords);
        // this is the correct robot pose
        assertEquals(1.36, robotInFieldCoords.getTranslation().getX(), kDelta); // <== THIS IS WHERE THE BUG IS
        assertEquals(6.75, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0.69, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI, robotInFieldCoords.getRotation().getZ(), kDelta);

        // just a projection of X and Y
        Pose2d robotIn2dFieldCords = robotInFieldCoords.toPose2d();
        assertEquals(1.36, robotIn2dFieldCords.getTranslation().getX(), kDelta);
        assertEquals(6.75, robotIn2dFieldCords.getTranslation().getY(), kDelta);
        assertEquals(Math.PI, robotIn2dFieldCords.getRotation().getRadians(), kDelta);
    }

    /**
     * Diagram for cases below.
     *
     * Test case where the robot and the tag are at opposite corners of a square.
     * The robot is at the origin, which is at the lower right corner, facing the
     * opposite corner. The tag is at the upper left, visible from below. Tag
     * orientation "into the page" is up.
     * 
     * .*
     * .* facing up
     * .*
     * TAG................. (1, 0)
     * ....................
     * ....................
     * ....................
     * ....................
     * .............*......
     * ...............*.... facing the tag
     * .................*..
     * .................ROBOT (0,0)
     * (1,0)
     * 
     * tag pose is R(0)|t(1,1)
     * robot pose is R(pi/4)|t(0,0)
     */
    @Test
    public void testOrientation() {
        // this is what the gyro says
        Rotation3d robotRotationInFieldCoords = new Rotation3d(0, 0, Math.PI / 4);
        // this is what the camera says
        Rotation3d tagRotationInCameraCoords = new Rotation3d(0, 0, -Math.PI / 4);
        // note that the camera and gyro are opposite.
        assertEquals(robotRotationInFieldCoords.times(-1.0).getZ(), tagRotationInCameraCoords.getZ());
        // this is the corrected pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));
        // calculate from the gyro
        Rotation3d tagRotationInRobotCoordsFromGyro = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                robotRotationInFieldCoords);
        // the gyro calc and the camera say the same thing.
        assertEquals(tagRotationInRobotCoordsFromGyro.getZ(), tagRotationInCameraCoords.getZ());
        // use the camera's opinion
        Rotation3d tagRotationInRobotCoords = tagRotationInCameraCoords;

        Translation3d tagTranslationInRobotCords = new Translation3d(Math.sqrt(2), 0, 0);

        Pose3d robotInFieldCoords = VisionDataProvider.toFieldCoordinates(
                tagRotationInRobotCoords,
                tagTranslationInRobotCords,
                tagInFieldCoords);
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    @Test
    public void testTagRotationInRobotCoordsFromGyro() {
        // this is what the gyro says
        Rotation3d robotRotationInFieldCoords = new Rotation3d(0, 0, Math.PI / 4);
        // this is the corrected tag pose
        Rotation3d tagRotationInFieldCoords = new Rotation3d(0, 0, 0);
        // calculate from the gyro
        Rotation3d tagRotationInRobotCoordsFromGyro = PoseEstimationHelper.tagRotationInRobotCoordsFromGyro(
                tagRotationInFieldCoords,
                robotRotationInFieldCoords);
        // this is what the camera says
        Rotation3d tagRotationInCameraCoords = new Rotation3d(0, 0, -Math.PI / 4);
        // the gyro calc and the camera say the same thing.
        assertEquals(tagRotationInRobotCoordsFromGyro.getZ(), tagRotationInCameraCoords.getZ());
    }
}
