package team100.localization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Static methods used to interpret camera input.
 */
public class PoseEstimationHelper {

    /**
     * given the gyro rotation and the camera offset, return the camera absolute
     * rotation. Package-private for testing.
     */
    static Rotation3d cameraRotationInFieldCoords(
            Transform3d cameraInRobotCoords,
            Rotation3d robotRotationInFieldCoordsFromGyro) {
        return cameraInRobotCoords.getRotation()
                .rotateBy(robotRotationInFieldCoordsFromGyro);
    }

    /**
     * Extract the translation from a "z-forward" blip and return the same
     * translation expressed in our usual "x-forward" NWU translation.
     * Package-private for testing.
     */
    static Translation3d blipToNWU(Blip b) {
        return new Translation3d(b.pose_t[2][0], -1.0 * b.pose_t[0][0], -1.0 * b.pose_t[1][0]);
    }

    /**
     * Because the camera's estimate of tag rotation isn't very accurate, this
     * synthesizes an estimate using the tag rotation in field frame (from json) and
     * the camera rotation in field frame (from gyro). Package-private for testing.
     */
    static Rotation3d tagRotationInRobotCoordsFromGyro(
            Rotation3d tagRotationInFieldCoords,
            Rotation3d cameraRotationInFieldCoords) {
        return tagRotationInFieldCoords.rotateBy(cameraRotationInFieldCoords.unaryMinus());
    }

    /**
     * Given the tag in camera frame and tag in field frame, return the camera in
     * field frame. Package-private for testing.
     */
    static Pose3d toFieldCoordinates(Transform3d tagInCameraCords, Pose3d tagInFieldCords) {
        // First invert the camera-to-tag transform, obtaining tag-to-camera.
        Transform3d cameraInTagCords = tagInCameraCords.inverse();
        // Then compose field-to-tag with tag-to-camera to get field-to-camera.
        return tagInFieldCords.transformBy(cameraInTagCords);
    }

    /**
     * Given the camera in field frame and camera in robot frame, return the robot
     * in field frame. Package-private for testing.
     */
    static Pose3d applyCameraOffset(Pose3d cameraInFieldCoords, Transform3d cameraInRobotCoords) {
        Transform3d robotInCameraCoords = cameraInRobotCoords.inverse();
        return cameraInFieldCoords.transformBy(robotInCameraCoords);
    }

    private PoseEstimationHelper() {
    }

}
