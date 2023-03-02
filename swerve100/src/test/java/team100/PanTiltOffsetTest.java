package team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import frc.robot.localization.VisionDataProvider;

/**
 * Figure out how to correct for pan and tilt
 *
 * Same diagram as VisionTransformTest.
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
public class PanTiltOffsetTest {
    private static final double kDelta = 0.01;

    /**
     * Start with camera and robot poses identical.
     */
    @Test
    public void testNoOffset() {

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // camera input
        Rotation3d unreliableTagRotationInCameraCoordsFromCamera = new Rotation3d(0, 0, -Math.PI / 4);
        Translation3d tagTranslationInCameraCoordsFromCamera = new Translation3d(Math.sqrt(2), 0, 0);

        // lookup the tag pose
        Pose3d tagInFieldCoordsFromLayout = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead:
        Rotation3d tagRotationInRobotCoordsFromGyro = VisionDataProvider.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoordsFromLayout.getRotation(),
                robotRotationInFieldCoordsFromGyro);

        // in this case the camera and robot rotations are the same
        assertEquals(tagRotationInRobotCoordsFromGyro.getZ(), unreliableTagRotationInCameraCoordsFromCamera.getZ());

        Pose3d robotInFieldCoords = VisionDataProvider.toFieldCoordinates(
                tagRotationInRobotCoordsFromGyro,
                tagTranslationInCameraCoordsFromCamera,
                tagInFieldCoordsFromLayout);
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);

    }

    /**
     * Correct for offset but the offset is zero.
     */
    @Test
    public void testZeroOffset() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(),
                new Rotation3d());

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);
        // Rotation3d cameraRotationInFieldCoords = robotRotationInFieldCoordsFromGyro
        // .plus(cameraInRobotCoords.getRotation());

        // camera input; we ignore the rotation from the camera.
        Translation3d tagTranslationInCameraCoords = new Translation3d(Math.sqrt(2), 0, 0);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = VisionDataProvider.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // check that the derived tag rotation is correct
        assertEquals(-Math.PI / 4, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        Pose3d cameraInFieldCoords = VisionDataProvider.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(0, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Dolly back 0.5m behind the robot center.
     */
    @Test
    public void testDolly() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(-0.5, 0, 0),
                new Rotation3d());

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);
        // Rotation3d cameraRotationInFieldCoords = robotRotationInFieldCoordsFromGyro
        // .plus(cameraInRobotCoords.getRotation());

        // camera input; we ignore the rotation from the camera.
        // because the camera happens to be pointing at the tag, the distance is simply
        // longer by the amount of the offset
        Translation3d tagTranslationInCameraCoords = new Translation3d(Math.sqrt(2) + 0.5, 0, 0);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = VisionDataProvider.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // check that the derived tag rotation is correct
        assertEquals(-Math.PI / 4, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        Pose3d cameraInFieldCoords = VisionDataProvider.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(-Math.sqrt(2) / 4, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(-Math.sqrt(2) / 4, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Pan 45 degrees to the left.
     */
    @Test
    public void testPan() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, 0, Math.PI / 4));

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);
        // Rotation3d cameraRotationInFieldCoords = robotRotationInFieldCoordsFromGyro
        // .plus(cameraInRobotCoords.getRotation());

        // camera input; we ignore the rotation from the camera.
        // because of the pan, the translation is different.
        Translation3d tagTranslationInCameraCoords = new Translation3d(1, -1, 0);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = VisionDataProvider.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // check that the derived tag rotation is correct
        // because of the pan, the rotation is different.
        assertEquals(-Math.PI / 2, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        // should be at the origin but looking 90 degrees left
        Pose3d cameraInFieldCoords = VisionDataProvider.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(0, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 2, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Truck sqrt(2)m left.
     */
    @Test
    public void testTruck() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(0, Math.sqrt(2), 0),
                new Rotation3d(0, 0, 0));

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);
        // Rotation3d cameraRotationInFieldCoords = robotRotationInFieldCoordsFromGyro
        // .plus(cameraInRobotCoords.getRotation());

        // camera input; we ignore the rotation from the camera.
        // because of the truck, the translation is different.
        Translation3d tagTranslationInCameraCoords = new Translation3d(Math.sqrt(2), -Math.sqrt(2), 0);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = VisionDataProvider.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // check that the derived tag rotation is correct
        // because of the pan, the rotation is different.
        assertEquals(-Math.PI / 4, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        // should be moved to the left
        Pose3d cameraInFieldCoords = VisionDataProvider.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(-1, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(1, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Moved and rotated, all in 2d.
     */
    @Test
    public void testAll2d() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(Math.sqrt(2), Math.sqrt(2), 0),
                new Rotation3d(0, 0, -Math.PI / 2));

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);
        // Rotation3d cameraRotationInFieldCoords = robotRotationInFieldCoordsFromGyro
        // .plus(cameraInRobotCoords.getRotation());

        // camera input; we ignore the rotation from the camera.
        // because of the truck, the translation is different.
        Translation3d tagTranslationInCameraCoords = new Translation3d(Math.sqrt(2), 0, 0);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = VisionDataProvider.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // check that the derived tag rotation is correct
        // because of the pan, the rotation is different.
        assertEquals(Math.PI / 4, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        // should be moved to the left
        Pose3d cameraInFieldCoords = VisionDataProvider.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(0, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(2, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(-Math.PI / 4, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Tilt 45 degrees up.
     */
    @Test
    public void testTilt() {
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, -Math.PI / 4, 0)); // upward tilt is negative pitch

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);
        // Rotation3d cameraRotationInFieldCoords = robotRotationInFieldCoordsFromGyro
        // .plus(cameraInRobotCoords.getRotation());

        // do we still get the correct pan and tilt values from the combined rotation?
        // the "normal" order is Tate-Bryant so yaw then pitch then roll.
        assertEquals(0, cameraRotationInFieldCoords.getX(), kDelta);
        assertEquals(-Math.PI / 4, cameraRotationInFieldCoords.getY(), kDelta);
        assertEquals(Math.PI / 4, cameraRotationInFieldCoords.getZ(), kDelta);

        // camera input; we ignore the rotation from the camera.
        // because of the tilt, the range (x) is more, and there's downward offset
        Translation3d tagTranslationInCameraCoords = new Translation3d(1, 0, -1);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = VisionDataProvider.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // do we get the right tag rotation?
        // negative roll in camera frame, not sure this is right.
        assertEquals(-0.615, tagRotationInCameraCoords.getX(), kDelta);
        // positive pitch in camera frame, not sure this is right.
        assertEquals(0.524, tagRotationInCameraCoords.getY(), kDelta);
        // negative yaw in camera frame, not sure this is right.
        assertEquals(-0.955, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        // should be at the origin, panned like the robot, and tilted 45 deg up
        Pose3d cameraInFieldCoords = VisionDataProvider.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(0, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        // upward tilt
        assertEquals(-Math.PI / 4, cameraInFieldCoords.getRotation().getY(), kDelta);
        // same yaw as robot
        assertEquals(Math.PI / 4, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * Offset to the left, pan to face the tag, and tilt 45 degrees up. tag view
     * should be a simple tilt from there.
     */
    @Test
    public void testTiltWithPandAndOffset() {
        // in robot coords the camera rotation is both tilt and pan
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(0, Math.sqrt(2), 0),
                new Rotation3d(0, -Math.PI / 4, -Math.PI / 4));

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 4);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);
        // Rotation3d cameraRotationInFieldCoords = robotRotationInFieldCoordsFromGyro
        // .plus(cameraInRobotCoords.getRotation());

        // do we still get the correct pan and tilt values from the combined rotation?
        // the "normal" order is Tate-Bryant so yaw then pitch then roll.
        // in field coords the camera rotation is pure tilt
        assertEquals(0, cameraRotationInFieldCoords.getX(), kDelta);
        assertEquals(-Math.PI / 4, cameraRotationInFieldCoords.getY(), kDelta);
        assertEquals(0, cameraRotationInFieldCoords.getZ(), kDelta);

        // camera input; we ignore the rotation from the camera.
        // because of the tilt, the range (x) is more, and there's downward offset
        Translation3d tagTranslationInCameraCoords = new Translation3d(Math.sqrt(2), 0, -Math.sqrt(2));

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = VisionDataProvider.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // do we get the right tag rotation?
        // should be positive pitch
        assertEquals(0, tagRotationInCameraCoords.getX(), kDelta); //
        assertEquals(Math.PI / 4, tagRotationInCameraCoords.getY(), kDelta);
        assertEquals(0, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        Pose3d cameraInFieldCoords = VisionDataProvider.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(-1, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(1, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(-Math.PI / 4, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(0, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

    /**
     * semi-realistic example
     * tag is at R(0,0,PI)t(1,4,1)
     * robot is at R(0,0,-3PI/4)t(3,3,0)
     * camera offset is R(0,-PI/4,-PI/4)t(0,-sqrt(2),1)
     * so tag in camera view should be R(0, PI/4, 0)t(sqrt(2)/2,0-sqrt(2)/2)
     */
    @Test
    public void testSemiRealisticExample() {
        // in robot coords the camera rotation is both tilt and pan
        Transform3d cameraInRobotCoords = new Transform3d(
                new Translation3d(0, -Math.sqrt(2), 1),
                new Rotation3d(0, -Math.PI / 4, -Math.PI / 4));

        // gyro input
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, -3.0 * Math.PI / 4.0);

        // we use the camera rotation to fix up the camera input
        // note the order here.
        Rotation3d cameraRotationInFieldCoords = cameraInRobotCoords.getRotation()
                .plus(robotRotationInFieldCoordsFromGyro);
        // Rotation3d cameraRotationInFieldCoords = robotRotationInFieldCoordsFromGyro
        // .plus(cameraInRobotCoords.getRotation());

        // do we still get the correct pan and tilt values from the combined rotation?
        // the "normal" order is Tate-Bryant so yaw then pitch then roll.
        // in field coords the camera rotation is pure tilt
        assertEquals(0, cameraRotationInFieldCoords.getX(), kDelta);
        assertEquals(-Math.PI / 4, cameraRotationInFieldCoords.getY(), kDelta);
        assertEquals(-Math.PI, cameraRotationInFieldCoords.getZ(), kDelta);

        // camera input; we ignore the rotation from the camera.
        // because of the tilt, the range (x) is more, and there's downward offset
        Translation3d tagTranslationInCameraCoords = new Translation3d(Math.sqrt(2)/2, 0, -Math.sqrt(2)/2);

        // lookup the tag pose
        Pose3d tagInFieldCoords = new Pose3d(1, 4, 1, new Rotation3d(0, 0, Math.PI));

        // don't trust the camera-derived rotation, calculate it instead from the gyro
        Rotation3d tagRotationInCameraCoords = VisionDataProvider.tagRotationInRobotCoordsFromGyro(
                tagInFieldCoords.getRotation(),
                cameraRotationInFieldCoords);

        // do we get the right tag rotation?
        // should be positive pitch
        assertEquals(0, tagRotationInCameraCoords.getX(), kDelta); //
        assertEquals(Math.PI / 4, tagRotationInCameraCoords.getY(), kDelta);
        assertEquals(0, tagRotationInCameraCoords.getZ(), kDelta);

        // now we have the corrected camera view
        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        // apply the inverted camera transform to the tag to get the camera pose
        Pose3d cameraInFieldCoords = VisionDataProvider.toFieldCoordinates(
                tagInCameraCoords,
                tagInFieldCoords);
        assertEquals(2, cameraInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(4, cameraInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(1, cameraInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, cameraInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(-Math.PI / 4, cameraInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(-Math.PI, cameraInFieldCoords.getRotation().getZ(), kDelta);

        // now apply the camera offset to get the robot pose.
        Pose3d robotInFieldCoords = cameraInFieldCoords.transformBy(cameraInRobotCoords.inverse());
        assertEquals(3, robotInFieldCoords.getTranslation().getX(), kDelta);
        assertEquals(3, robotInFieldCoords.getTranslation().getY(), kDelta);
        assertEquals(0, robotInFieldCoords.getTranslation().getZ(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getX(), kDelta);
        assertEquals(0, robotInFieldCoords.getRotation().getY(), kDelta);
        assertEquals(-3.0 * Math.PI / 4, robotInFieldCoords.getRotation().getZ(), kDelta);
    }

}
